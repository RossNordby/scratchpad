using BEPUutilities2;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct BodyPoses
    {
        public Vector3Wide Position;
        //Note that we store a quaternion rather than a matrix3x3. While this often requires some overhead when performing vector transforms or extracting basis vectors, 
        //systems needing to interact directly with this representation are often terrifically memory bound. Spending the extra ALU time to convert to a basis can actually be faster
        //than loading the extra 5 elements needed to express the full 3x3 rotation matrix. Also, it's marginally easier to keep the rotation normalized over time.
        //There may be an argument for the matrix variant to ALSO be stored for some bandwidth-unconstrained stages, but don't worry about that until there's a reason to worry about it.
        public QuaternionWide Orientation;
    }

    public struct BodyVelocities
    {
        public Vector3Wide LinearVelocity;
        public Vector3Wide AngularVelocity;
    }

    public struct BodyInertias
    {
        public Matrix3x3Wide InverseInertiaTensor;
        //Note that the inverse mass is included in the BodyInertias bundle. InverseMass is rotationally invariant, so it doesn't need to be updated...
        //But it's included alongside the rotated inertia tensor because to split it out would require that constraint presteps suffer another cache miss when they
        //gather the inverse mass in isolation. (From the solver's perspective, inertia/mass gathering is incoherent.)
        public Vector<float> InverseMass;
    }

    public struct BodyPose
    {
        public Vector3 Position;
        public BEPUutilities2.Quaternion Orientation;
    }

    public struct BodyVelocity
    {
        public Vector3 Linear;
        public Vector3 Angular;
    }
    public struct BodyInertia
    {
        public Matrix3x3 InverseInertiaTensor;
        public float InverseMass;
    }
    public struct BodyDescription
    {
        public BodyPose Pose;
        public BodyInertia LocalInertia;
        public BodyVelocity Velocity;
    }
    /// <summary>
    /// Collection of allocated bodies.
    /// </summary>
    public class Bodies
    {
        //TODO: there is a somewhat weak argument suggesting we should use bufferpooled memory always.
        //That would mean tearing down a simulation would result in only a few objects being thrown away- constraint batches, the individual class stages, and so on.
        //In other words, the amount of garbage generated would be miniscule.
        //A slightly stronger argument is that array resizing can create fairly significant garbage blobs in the current setup.
        //If we kept that memory around, we could use it for other stuff. But it's questionable how often you'd actually have use for a megabyte long ephemeral array...
        //Maybe in the constraint batches or something. As long as you're able to compact the BufferPool, there's not much of an issue.

        /// <summary>
        /// Remaps a body handle to the actual array index of the body.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public Buffer<int> HandleToIndex;
        /// <summary>
        /// Remaps a body index to its handle.
        /// </summary>
        public Buffer<int> IndexToHandle;

        public Buffer<BodyPoses> Poses;
        public Buffer<BodyVelocities> Velocities;
        public Buffer<BodyInertias> LocalInertias;
        /// <summary>
        /// The world transformed inertias of bodies as of the last update. Note that this is not automatically updated for direct orientation changes or for body memory moves.
        /// It is only updated once during the frame. It should be treated as ephemeral information.
        /// </summary>
        internal Buffer<BodyInertias> Inertias;
        public IdPool IdPool;
        BufferPool pool;
        public int BodyCount;
        /// <summary>
        /// Gets the number of body bundles. Any trailing partial bundle is counted as a full bundle.
        /// </summary>
        public int BodyBundleCount
        {
            get
            {
                return BundleIndexing.GetBundleCount(BodyCount);
            }
        }

        public unsafe Bodies(BufferPool pool, int initialCapacity = 4096)
        {
            this.pool = pool;
            var initialCapacityInBundles = BundleIndexing.GetBundleCount(initialCapacity);
            InternalResize(initialCapacity);

            IdPool = new IdPool(initialCapacity);
        }

        unsafe void InternalResize(int targetBodyCapacity)
        {
            Debug.Assert(targetBodyCapacity > 0, "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
            var targetBundleCapacity = BundleIndexing.GetBundleCount(targetBodyCapacity);
            Debug.Assert(Poses.Length != BufferPool<BodyPoses>.GetLowestContainingElementCount(targetBundleCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.SpecializeFor<BodyPoses>().Take(targetBundleCapacity, out var newPoses);
            pool.SpecializeFor<BodyVelocities>().Take(targetBundleCapacity, out var newVelocities);
            pool.SpecializeFor<BodyInertias>().Take(targetBundleCapacity, out var newLocalInertias);
            pool.SpecializeFor<BodyInertias>().Take(targetBundleCapacity, out var newInertias);
            pool.SpecializeFor<int>().Take(targetBundleCapacity, out var newHandleToIndex);
            pool.SpecializeFor<int>().Take(targetBundleCapacity, out var newIndexToHandle);
            if (Poses.Length > 0)
            {
                Debug.Assert(Velocities.Length > 0 && LocalInertias.Length > 0 && Inertias.Length > 0 && HandleToIndex.Length > 0 && IndexToHandle.Length > 0,
                    "While individual capacities may differ, if any buffer has nonzero length, all should.");
                var bundleCount = BodyBundleCount;
                Poses.CopyTo(0, ref newPoses, 0, bundleCount);
                Velocities.CopyTo(0, ref newVelocities, 0, bundleCount);
                LocalInertias.CopyTo(0, ref newLocalInertias, 0, bundleCount);
                Inertias.CopyTo(0, ref newInertias, 0, bundleCount);
                IndexToHandle.CopyTo(0, ref newIndexToHandle, 0, bundleCount);
                HandleToIndex.CopyTo(0, ref newHandleToIndex, 0, bundleCount);
                ReturnBodyResources();
            }
            Poses = newPoses;
            Velocities = newVelocities;
            Inertias = newInertias;
            LocalInertias = newLocalInertias;
            IndexToHandle = newIndexToHandle;
            HandleToIndex = newHandleToIndex;
            //Initialize all the indices beyond the copied region to -1.
            Unsafe.InitBlock(((int*)HandleToIndex.Memory) + BodyCount, 0xFF, (uint)(sizeof(int) * (HandleToIndex.Length - BodyCount)));
            Unsafe.InitBlock(((int*)IndexToHandle.Memory) + BodyCount, 0xFF, (uint)(sizeof(int) * (IndexToHandle.Length - BodyCount)));
            //Note that we do NOT modify the idpool's internal queue size here. We lazily handle that during adds, and during explicit calls to EnsureCapacity, Compact, and Resize.
            //The idpool's internal queue will often be nowhere near as large as the actual body size except in corner cases, so in the usual case, being lazy saves a little space.
            //If the user wants to guarantee zero resizes, EnsureCapacity provides them the option to do so.
        }

        public unsafe int Add(ref BodyDescription bodyDescription)
        {
            if (BodyCount == HandleToIndex.Length)
            {
                Debug.Assert(HandleToIndex.Length > 0, "The backing memory of the bodies set should be initialized before use. Did you dispose and then not call EnsureCapacity/Resize?");
                //Out of room; need to resize.
                var newSize = HandleToIndex.Length << 1;
                InternalResize(newSize);
            }
            var handle = IdPool.Take();
            var index = BodyCount++;
            HandleToIndex[handle] = index;
            IndexToHandle[index] = handle;

            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var indexInBundle);
            SetLane(ref Poses[bundleIndex], indexInBundle, ref bodyDescription.Pose);
            SetLane(ref Velocities[bundleIndex], indexInBundle, ref bodyDescription.Velocity);
            SetLane(ref LocalInertias[bundleIndex], indexInBundle, ref bodyDescription.LocalInertia);
            //TODO: Should the world inertias be updated on add? That would suggest a convention of also updating world inertias on any orientation change, which might not be wise given the API.

            return handle;
        }

        /// <summary>
        /// Removes a body from the set and returns whether a move occurred. If another body took its place, the move is output.
        /// </summary>
        /// <param name="handle">Handle of the body to remove.</param>
        /// <param name="removedIndex">Former index of the body that was removed.</param>
        /// <param name="movedBodyOriginalIndex">Original index of the body that was moved into the removed body's slot. -1 if no body had to be moved.</param>
        /// <returns>True if a body was moved, false otherwise.</returns>
        public bool Remove(int handle, out int removedIndex, out int movedBodyOriginalIndex)
        {
            ValidateExistingHandle(handle);
            removedIndex = HandleToIndex[handle];

            //Move the last body into the removed slot.
            //This does introduce disorder- there may be value in a second overload that preserves order, but it would require large copies.
            //In the event that so many adds and removals are performed at once that they destroy contiguity, it may be better to just
            //explicitly sort after the fact rather than attempt to retain contiguity incrementally. Handle it as a batch, in other words.
            --BodyCount;
            bool bodyMoved = removedIndex < BodyCount;
            if (bodyMoved)
            {
                movedBodyOriginalIndex = BodyCount;
                //Copy the memory state of the last element down.
                BundleIndexing.GetBundleIndices(removedIndex, out var targetBundle, out var targetInner);
                BundleIndexing.GetBundleIndices(movedBodyOriginalIndex, out var sourceBundle, out var sourceInner);
                GatherScatter.CopyLane(ref Poses[sourceBundle], sourceInner, ref Poses[targetBundle], targetInner);
                GatherScatter.CopyLane(ref Velocities[sourceBundle], sourceInner, ref Velocities[targetBundle], targetInner);
                GatherScatter.CopyLane(ref LocalInertias[sourceBundle], sourceInner, ref LocalInertias[targetBundle], targetInner);
                //Note that if you ever treat the world inertias as 'always updated', it would need to be copied here.
                //Point the body handles at the new location.
                var lastHandle = IndexToHandle[movedBodyOriginalIndex];
                HandleToIndex[lastHandle] = removedIndex;
                IndexToHandle[removedIndex] = lastHandle;
                IndexToHandle[movedBodyOriginalIndex] = -1;

            }
            else
            {
                movedBodyOriginalIndex = -1;
            }
            IdPool.Return(handle);
            HandleToIndex[handle] = -1;
            return bodyMoved;
        }

        [Conditional("DEBUG")]
        internal void ValidateHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle >= HandleToIndex.Length || HandleToIndex[handle] < 0 || IndexToHandle[HandleToIndex[handle]] == handle,
                "If a handle exists, both directions should match.");
        }
        [Conditional("DEBUG")]
        internal void ValidateExistingHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle < HandleToIndex.Length && HandleToIndex[handle] >= 0 && IndexToHandle[HandleToIndex[handle]] == handle,
                "This body handle doesn't seem to exist, or the mappings are out of sync. If a handle exists, both directions should match.");
        }
        public bool Contains(int handle)
        {
            ValidateHandle(handle);
            return handle < HandleToIndex.Length && HandleToIndex[handle] >= 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void GetBundleIndices(int handle, out int bundleIndex, out int innerIndex)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle];
            BundleIndexing.GetBundleIndices(index, out bundleIndex, out innerIndex);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetLane(ref BodyInertias targetBundle, int innerIndex, ref BodyInertia inertia)
        {
            ref var targetLane = ref GatherScatter.Get(ref targetBundle.InverseInertiaTensor.X.X, innerIndex);

            targetLane = inertia.InverseInertiaTensor.X.X;
            Unsafe.Add(ref targetLane, Vector<float>.Count) = inertia.InverseInertiaTensor.X.Y;
            Unsafe.Add(ref targetLane, 2 * Vector<float>.Count) = inertia.InverseInertiaTensor.X.Z;
            Unsafe.Add(ref targetLane, 3 * Vector<float>.Count) = inertia.InverseInertiaTensor.Y.X;
            Unsafe.Add(ref targetLane, 4 * Vector<float>.Count) = inertia.InverseInertiaTensor.Y.Y;
            Unsafe.Add(ref targetLane, 5 * Vector<float>.Count) = inertia.InverseInertiaTensor.Y.Z;
            Unsafe.Add(ref targetLane, 6 * Vector<float>.Count) = inertia.InverseInertiaTensor.Z.X;
            Unsafe.Add(ref targetLane, 7 * Vector<float>.Count) = inertia.InverseInertiaTensor.Z.Y;
            Unsafe.Add(ref targetLane, 8 * Vector<float>.Count) = inertia.InverseInertiaTensor.Z.Z;
            Unsafe.Add(ref targetLane, 9 * Vector<float>.Count) = inertia.InverseMass;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLane(ref BodyInertias targetBundle, int innerIndex, out BodyInertia inertia)
        {
            ref var sourceLane = ref GatherScatter.Get(ref targetBundle.InverseInertiaTensor.X.X, innerIndex);

            inertia.InverseInertiaTensor.X = new Vector3(
                sourceLane,
                Unsafe.Add(ref sourceLane, Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 2 * Vector<float>.Count));
            inertia.InverseInertiaTensor.Y = new Vector3(
                Unsafe.Add(ref sourceLane, 3 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 4 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 5 * Vector<float>.Count));
            inertia.InverseInertiaTensor.Z = new Vector3(
                Unsafe.Add(ref sourceLane, 6 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 7 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 8 * Vector<float>.Count));
            inertia.InverseMass =
                Unsafe.Add(ref sourceLane, 9 * Vector<float>.Count);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void SetLane(ref BodyVelocities targetBundle, int innerIndex, ref BodyVelocity velocity)
        {
            ref var targetLane = ref GatherScatter.Get(ref targetBundle.LinearVelocity.X, innerIndex);

            targetLane = velocity.Linear.X;
            Unsafe.Add(ref targetLane, Vector<float>.Count) = velocity.Linear.Y;
            Unsafe.Add(ref targetLane, 2 * Vector<float>.Count) = velocity.Linear.Z;
            Unsafe.Add(ref targetLane, 3 * Vector<float>.Count) = velocity.Angular.X;
            Unsafe.Add(ref targetLane, 4 * Vector<float>.Count) = velocity.Angular.Y;
            Unsafe.Add(ref targetLane, 5 * Vector<float>.Count) = velocity.Angular.Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GetLane(ref BodyVelocities targetBundle, int innerIndex, out BodyVelocity velocity)
        {
            ref var sourceLane = ref GatherScatter.Get(ref targetBundle.LinearVelocity.X, innerIndex);

            velocity.Linear = new Vector3(
                sourceLane,
                Unsafe.Add(ref sourceLane, Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 2 * Vector<float>.Count));
            velocity.Angular = new Vector3(
                Unsafe.Add(ref sourceLane, 3 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 4 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 5 * Vector<float>.Count));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void SetLane(ref BodyPoses targetBundle, int innerIndex, ref BodyPose pose)
        {
            ref var targetLane = ref GatherScatter.Get(ref targetBundle.Position.X, innerIndex);

            targetLane = pose.Position.X;
            Unsafe.Add(ref targetLane, Vector<float>.Count) = pose.Position.Y;
            Unsafe.Add(ref targetLane, 2 * Vector<float>.Count) = pose.Position.Z;
            Unsafe.Add(ref targetLane, 3 * Vector<float>.Count) = pose.Orientation.X;
            Unsafe.Add(ref targetLane, 4 * Vector<float>.Count) = pose.Orientation.Y;
            Unsafe.Add(ref targetLane, 5 * Vector<float>.Count) = pose.Orientation.Z;
            Unsafe.Add(ref targetLane, 6 * Vector<float>.Count) = pose.Orientation.W;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GetLane(ref BodyPoses targetBundle, int innerIndex, out BodyPose pose)
        {
            ref var sourceLane = ref GatherScatter.Get(ref targetBundle.Position.X, innerIndex);

            pose.Position = new Vector3(
                sourceLane,
                Unsafe.Add(ref sourceLane, Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 2 * Vector<float>.Count));
            pose.Orientation = new BEPUutilities2.Quaternion(
                Unsafe.Add(ref sourceLane, 3 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 4 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 5 * Vector<float>.Count),
                Unsafe.Add(ref sourceLane, 6 * Vector<float>.Count));
        }

        public void SetPose(int handle, ref BodyPose pose)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            SetLane(ref Poses[bundleIndex], innerIndex, ref pose);
        }
        public void GetPose(int handle, out BodyPose pose)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GetLane(ref Poses[bundleIndex], innerIndex, out pose);
        }
        public void SetVelocity(int handle, ref BodyVelocity velocity)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            SetLane(ref Velocities[bundleIndex], innerIndex, ref velocity);
        }
        public void GetVelocity(int handle, out BodyVelocity velocity)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GetLane(ref Velocities[bundleIndex], innerIndex, out velocity);
        }
        public void SetLocalInertia(int handle, ref BodyInertia inertia)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            SetLane(ref LocalInertias[bundleIndex], innerIndex, ref inertia);
        }
        public void GetLocalInertia(int handle, out BodyInertia inertia)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GetLane(ref LocalInertias[bundleIndex], innerIndex, out inertia);
        }

        /// <summary>
        /// Gets a value roughly representing the amount of energy in the simulation. This is occasionally handy for debug purposes.
        /// </summary>
        public float GetBodyEnergyHeuristic()
        {
            var lastBundleIndex = (BodyCount - 1) >> BundleIndexing.VectorShift;
            //Mask away the unused lanes. We're modifying the actual velocities; that's valid because they're unused.
            var lastBundleCount = BodyCount - (lastBundleIndex << BundleIndexing.VectorShift);
            var zeroVelocity = new BodyVelocity();
            for (int i = lastBundleCount; i < Vector<float>.Count; ++i)
            {
                SetLane(ref Velocities[lastBundleIndex], lastBundleCount, ref zeroVelocity);
            }
            Vector<float> accumulated = Vector<float>.Zero;
            for (int bundleIndex = 0; bundleIndex <= lastBundleIndex; ++bundleIndex)
            {
                Vector3Wide.Dot(ref Velocities[bundleIndex].LinearVelocity, ref Velocities[bundleIndex].LinearVelocity, out var linearDot);
                Vector3Wide.Dot(ref Velocities[bundleIndex].AngularVelocity, ref Velocities[bundleIndex].AngularVelocity, out var angularDot);
                accumulated += linearDot + angularDot;
            }
            return Vector.Dot(accumulated, Vector<float>.One);
        }


        /// <summary>
        /// Swaps the memory of two bodies. Indexed by memory slot, not by handle index.
        /// </summary>
        /// <param name="slotA">Memory slot of the first body to swap.</param>
        /// <param name="slotB">Memory slot of the second body to swap.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Swap(int slotA, int slotB)
        {
            HandleToIndex[IndexToHandle[slotA]] = slotB;
            HandleToIndex[IndexToHandle[slotB]] = slotA;
            var oldHandleA = IndexToHandle[slotA];
            IndexToHandle[slotA] = IndexToHandle[slotB];
            IndexToHandle[slotB] = oldHandleA;
            BundleIndexing.GetBundleIndices(slotA, out var bundleA, out var innerA);
            BundleIndexing.GetBundleIndices(slotB, out var bundleB, out var innerB);
            GatherScatter.SwapLanes(ref Poses[bundleA], innerA, ref Poses[bundleB], innerB);
            GatherScatter.SwapLanes(ref Velocities[bundleA], innerA, ref Velocities[bundleB], innerB);
            GatherScatter.SwapLanes(ref LocalInertias[bundleA], innerA, ref LocalInertias[bundleB], innerB);
        }


        /// <summary>
        /// Clears all bodies from the set without returning any memory to the pool.
        /// </summary>
        public void Clear()
        {
            //Well that's pretty easy.
            BodyCount = 0;
        }


        /// <summary>
        /// Returns the currently used resources.
        /// </summary>
        private void ReturnBodyResources()
        {
            pool.SpecializeFor<BodyPoses>().Return(ref Poses);
            pool.SpecializeFor<BodyVelocities>().Return(ref Velocities);
            pool.SpecializeFor<BodyInertias>().Return(ref LocalInertias);
            pool.SpecializeFor<BodyInertias>().Return(ref Inertias);
            pool.SpecializeFor<int>().Return(ref HandleToIndex);
            pool.SpecializeFor<int>().Return(ref IndexToHandle);
        }

        /// <summary>
        /// Returns all body resources to the pool used to create them.
        /// </summary>
        /// <remarks>The object can be reused if it is reinitialized by using EnsureCapacity or Resize.</remarks>
        public void Dispose()
        {
            ReturnBodyResources();
            //Zeroing the lengths is useful for reuse- resizes know not to copy old invalid data.
            Poses = new Buffer<BodyPoses>();
            Velocities = new Buffer<BodyVelocities>();
            LocalInertias = new Buffer<BodyInertias>();
            Inertias = new Buffer<BodyInertias>();
            HandleToIndex = new Buffer<int>();
            IndexToHandle = new Buffer<int>();
            IdPool.Dispose();
        }
    }
}
