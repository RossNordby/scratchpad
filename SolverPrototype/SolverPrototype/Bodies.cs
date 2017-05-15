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
        public Vector<float> InverseMass;
    }
    
    public struct BodyDescription
    {
        public BodyInertia LocalInertia;
        public BodyVelocity Velocity;
    }
    public struct BodyInertia
    {
        public Matrix3x3 InverseInertiaTensor;
        public float InverseMass;
    }
    public struct BodyVelocity
    {
        public Vector3 Linear;
        public Vector3 Angular;
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
        public int[] HandleToIndex;
        /// <summary>
        /// Remaps a body index to its handle.
        /// </summary>
        public int[] IndexToHandle;

        public BodyPoses[] Poses;
        public BodyVelocities[] Velocities;
        public BodyInertias[] LocalInertias;
        public BodyInertias[] Inertias;
        //TODO: While our current tests do not actually integrate orientation, when we do, we'll need to also update the world inertia.
        //The constraints will need to gather from the transformed inertia rather than from the local variants.
        //Note that the inverse mass is included in the BodyInertias bundle. InverseMass is rotationally invariant, so it doesn't need to be updated...
        //But it's included alongside the rotated inertia tensor because to split it out would require that constraint presteps suffer another cache miss when they
        //gather the inverse mass in isolation. (From the solver's perspective, inertia/mass gathering is incoherent.)
        //public BodyInertias[] InertiaBundles;
        public IdPool IdPool;
        public int BodyCount;
        /// <summary>
        /// Gets the number of body bundles. Any trailing partial bundle is counted as a full bundle.
        /// </summary>
        public int BodyBundleCount
        {
            get
            {
                var bundleCount = BodyCount >> BundleIndexing.VectorShift;
                if ((bundleCount << BundleIndexing.VectorShift) < BodyCount)
                    ++bundleCount;
                return bundleCount;
            }
        }

        public unsafe Bodies(int initialCapacity = 4096)
        {
            var initialCapacityInBundles = BundleIndexing.GetBundleCount(initialCapacity);
            Velocities = new BodyVelocities[initialCapacityInBundles];
            LocalInertias = new BodyInertias[initialCapacityInBundles];

            IdPool = new IdPool(initialCapacity);
            HandleToIndex = new int[initialCapacity];
            IndexToHandle = new int[initialCapacity];
            //Initialize all the indices to -1.
            InitializeIndices(HandleToIndex, 0, HandleToIndex.Length);
            InitializeIndices(IndexToHandle, 0, IndexToHandle.Length);
        }

        unsafe static void InitializeIndices(int[] array, int start, int count)
        {
            fixed (int* pointer = &array[start])
            {
                Unsafe.InitBlock(pointer, 0xFF, (uint)(sizeof(int) * count));
            }
        }

        public unsafe int Add(ref BodyDescription bodyDescription)
        {
            if (BodyCount == HandleToIndex.Length)
            {
                //Out of room; need to resize.
                var newSize = HandleToIndex.Length << 1;
                Array.Resize(ref Velocities, newSize);
                Array.Resize(ref LocalInertias, newSize);
                Array.Resize(ref IndexToHandle, newSize);
                Array.Resize(ref HandleToIndex, newSize);

                InitializeIndices(HandleToIndex, BodyCount, BodyCount);
                InitializeIndices(IndexToHandle, BodyCount, BodyCount);
            }
            var handle = IdPool.Take();
            var index = BodyCount++;
            HandleToIndex[handle] = index;
            IndexToHandle[index] = handle;

            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var indexInBundle);
            GatherScatter.SetLane(ref LocalInertias[bundleIndex], indexInBundle, ref bodyDescription.LocalInertia);
            GatherScatter.SetLane(ref Velocities[bundleIndex], indexInBundle, ref bodyDescription.Velocity);

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
                GatherScatter.CopyLane(ref Velocities[sourceBundle], sourceInner, ref Velocities[targetBundle], targetInner);
                GatherScatter.CopyLane(ref LocalInertias[sourceBundle], sourceInner, ref LocalInertias[targetBundle], targetInner);
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
        public void SetVelocity(int handle, ref BodyVelocity velocity)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.SetLane(ref Velocities[bundleIndex], innerIndex, ref velocity);
        }
        public void GetVelocity(int handle, out BodyVelocity velocity)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.GetLane(ref Velocities[bundleIndex], innerIndex, out velocity);
        }
        public void SetLocalInertia(int handle, ref BodyInertia inertia)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.SetLane(ref LocalInertias[bundleIndex], innerIndex, ref inertia);
        }
        public void GetLocalInertia(int handle, out BodyInertia inertia)
        {
            ValidateExistingHandle(handle);
            GetBundleIndices(handle, out var bundleIndex, out var innerIndex);
            GatherScatter.GetLane(ref LocalInertias[bundleIndex], innerIndex, out inertia);
        }

        public float GetBodyEnergyHeuristic()
        {
            float accumulated = 0;
            var lastBundleIndex = (BodyCount - 1) >> BundleIndexing.VectorShift;
            //Mask away the unused lanes. We're modifying the actual velocities; that's valid because they're unused.
            var lastBundleCount = BodyCount - (lastBundleIndex << BundleIndexing.VectorShift);
            var zeroVelocity = new BodyVelocity();
            for (int i = lastBundleCount; i < Vector<float>.Count; ++i)
            {
                GatherScatter.SetLane(ref Velocities[lastBundleIndex], lastBundleCount, ref zeroVelocity);
            }
            for (int bundleIndex = 0; bundleIndex <= lastBundleIndex; ++bundleIndex)
            {
                Vector3Wide.Dot(ref Velocities[bundleIndex].LinearVelocity, ref Velocities[bundleIndex].LinearVelocity, out var linearDot);
                Vector3Wide.Dot(ref Velocities[bundleIndex].AngularVelocity, ref Velocities[bundleIndex].AngularVelocity, out var angularDot);

                accumulated += Vector.Dot(linearDot, Vector<float>.One) + Vector.Dot(angularDot, Vector<float>.One);
            }
            return accumulated;
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
            GatherScatter.SwapLanes(ref Velocities[bundleA], innerA, ref Velocities[bundleB], innerB);
            GatherScatter.SwapLanes(ref LocalInertias[bundleA], innerA, ref LocalInertias[bundleB], innerB);
        }
    }
}
