using BEPUutilities2;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using SolverPrototype.Constraints;
using SolverPrototype.Collidables;
using BEPUutilities2.Collections;

namespace SolverPrototype
{

    /// <summary>
    /// Collection of allocated bodies.
    /// </summary>
    public class Bodies
    {
        /// <summary>
        /// Remaps a body handle to the actual array index of the body.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public Buffer<int> HandleToIndex;
        /// <summary>
        /// Remaps a body index to its handle.
        /// </summary>
        public Buffer<int> IndexToHandle;
        /// <summary>
        /// The set of collidables owned by each body. Speculative margins, continuity settings, and shape indices can be changed directly.
        /// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase of the collidable addition or removal.
        /// </summary>
        public Buffer<Collidable> Collidables;

        public Buffer<BodyPoses> Poses;
        public Buffer<BodyVelocities> Velocities;
        public Buffer<BodyInertias> LocalInertias;
        /// <summary>
        /// The world transformed inertias of bodies as of the last update. Note that this is not automatically updated for direct orientation changes or for body memory moves.
        /// It is only updated once during the frame. It should be treated as ephemeral information.
        /// </summary>
        public Buffer<BodyInertias> Inertias;
        public IdPool<Buffer<int>, BufferPool<int>> IdPool;
        protected BufferPool pool;
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

            IdPool = new IdPool<Buffer<int>, BufferPool<int>>(pool.SpecializeFor<int>(), initialCapacity);
        }
        
        unsafe void InternalResize(int targetBodyCapacity)
        {
            //There's no point in going smaller than the bundle size. That would result in resizes that don't resize any of the bundle structures.
            if (targetBodyCapacity < Vector<float>.Count)
                targetBodyCapacity = Vector<float>.Count;
            Debug.Assert(targetBodyCapacity > 0, "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
            //Note that we base the bundle capacities on the body capacity. This simplifies the conditions on allocation
            targetBodyCapacity = BufferPool<int>.GetLowestContainingElementCount(targetBodyCapacity);
            var targetBundleCapacity = BundleIndexing.GetBundleCount(targetBodyCapacity);
            Debug.Assert(Poses.Length != BufferPool<BodyPoses>.GetLowestContainingElementCount(targetBundleCapacity), "Should not try to use internal resize of the result won't change the size.");
            var bodyBundleCount = BodyBundleCount;
            pool.SpecializeFor<BodyPoses>().Resize(ref Poses, targetBundleCapacity, bodyBundleCount);
            pool.SpecializeFor<BodyVelocities>().Resize(ref Velocities, targetBundleCapacity, bodyBundleCount);
            pool.SpecializeFor<BodyInertias>().Resize(ref LocalInertias, targetBundleCapacity, bodyBundleCount);
            pool.SpecializeFor<BodyInertias>().Resize(ref Inertias, targetBundleCapacity, bodyBundleCount);
            pool.SpecializeFor<int>().Resize(ref IndexToHandle, targetBodyCapacity, BodyCount);
            pool.SpecializeFor<int>().Resize(ref HandleToIndex, targetBodyCapacity, BodyCount);
            pool.SpecializeFor<Collidable>().Resize(ref Collidables, targetBodyCapacity, BodyCount);
            //Initialize all the indices beyond the copied region to -1.
            Unsafe.InitBlock(((int*)HandleToIndex.Memory) + BodyCount, 0xFF, (uint)(sizeof(int) * (HandleToIndex.Length - BodyCount)));
            Unsafe.InitBlock(((int*)IndexToHandle.Memory) + BodyCount, 0xFF, (uint)(sizeof(int) * (IndexToHandle.Length - BodyCount)));
            //Collidables beyond the body count should all point to nothing, which corresponds to zero.
            Collidables.Clear(BodyCount, Collidables.Length - BodyCount);
            //Note that we do NOT modify the idpool's internal queue size here. We lazily handle that during adds, and during explicit calls to EnsureCapacity, Compact, and Resize.
            //The idpool's internal queue will often be nowhere near as large as the actual body size except in corner cases, so in the usual case, being lazy saves a little space.
            //If the user wants to guarantee zero resizes, EnsureCapacity provides them the option to do so.
        }

        public unsafe int Add(ref BodyDescription bodyDescription)
        {
            if (BodyCount == HandleToIndex.Length)
            {
                Debug.Assert(HandleToIndex.Allocated, "The backing memory of the bodies set should be initialized before use. Did you dispose and then not call EnsureCapacity/Resize?");
                //Out of room; need to resize.
                var newSize = HandleToIndex.Length << 1;
                InternalResize(newSize);
            }
            Debug.Assert(Math.Abs(bodyDescription.Pose.Orientation.Length() - 1) < 1e-6f, "Orientation should be initialized to a unit length quaternion.");
            var handle = IdPool.Take();
            var index = BodyCount++;
            HandleToIndex[handle] = index;
            IndexToHandle[index] = handle;
            ref var collidable = ref Collidables[index];
            collidable.Shape = bodyDescription.Collidable.Shape;
            collidable.Continuity = bodyDescription.Collidable.Continuity;
            collidable.SpeculativeMargin = bodyDescription.Collidable.SpeculativeMargin;
            //Collidable's broad phase index is left unset. The simulation is responsible for attaching that data.

            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var indexInBundle);
            SetLane(ref Poses[bundleIndex], indexInBundle, ref bodyDescription.Pose);
            SetLane(ref Velocities[bundleIndex], indexInBundle, ref bodyDescription.Velocity);
            SetLane(ref LocalInertias[bundleIndex], indexInBundle, ref bodyDescription.LocalInertia);
            //TODO: Should the world inertias be updated on add? That would suggest a convention of also updating world inertias on any orientation change, which might not be wise given the API.
            return handle;
        }

        /// <summary>
        /// Removes a body from the set by index and returns whether a move occurred. If another body took its place, the move is output.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to remove.</param>
        /// <param name="movedBodyOriginalIndex">Original index of the body that was moved into the removed body's slot. -1 if no body had to be moved.</param>
        /// <returns>True if a body was moved, false otherwise.</returns>
        public bool RemoveAt(int bodyIndex, out int movedBodyOriginalIndex)
        {
            Debug.Assert(bodyIndex >= 0 && bodyIndex < BodyCount);
            var handle = IndexToHandle[bodyIndex];
            //Move the last body into the removed slot.
            //This does introduce disorder- there may be value in a second overload that preserves order, but it would require large copies.
            //In the event that so many adds and removals are performed at once that they destroy contiguity, it may be better to just
            //explicitly sort after the fact rather than attempt to retain contiguity incrementally. Handle it as a batch, in other words.
            --BodyCount;
            bool bodyMoved = bodyIndex < BodyCount;
            if (bodyMoved)
            {
                movedBodyOriginalIndex = BodyCount;
                //Copy the memory state of the last element down.
                BundleIndexing.GetBundleIndices(bodyIndex, out var targetBundle, out var targetInner);
                BundleIndexing.GetBundleIndices(movedBodyOriginalIndex, out var sourceBundle, out var sourceInner);
                GatherScatter.CopyLane(ref Poses[sourceBundle], sourceInner, ref Poses[targetBundle], targetInner);
                GatherScatter.CopyLane(ref Velocities[sourceBundle], sourceInner, ref Velocities[targetBundle], targetInner);
                GatherScatter.CopyLane(ref LocalInertias[sourceBundle], sourceInner, ref LocalInertias[targetBundle], targetInner);
                //Note that if you ever treat the world inertias as 'always updated', it would need to be copied here.
                Collidables[bodyIndex] = Collidables[movedBodyOriginalIndex];
                //Point the body handles at the new location.
                var lastHandle = IndexToHandle[movedBodyOriginalIndex];
                HandleToIndex[lastHandle] = bodyIndex;
                IndexToHandle[bodyIndex] = lastHandle;

            }
            else
            {
                movedBodyOriginalIndex = -1;
            }
            //We rely on the collidable references being nonexistent beyond the body count.
            Collidables[BodyCount] = new Collidable();
            //The indices should also be set to all -1's beyond the body count.
            IndexToHandle[BodyCount] = -1;
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
        public void ValidateExistingHandle(int handle)
        {
            Debug.Assert(handle >= 0, "Handles must be nonnegative.");
            Debug.Assert(handle < HandleToIndex.Length && HandleToIndex[handle] >= 0 && IndexToHandle[HandleToIndex[handle]] == handle,
                "This body handle doesn't seem to exist, or the mappings are out of sync. If a handle exists, both directions should match.");
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
            return RemoveAt(removedIndex, out movedBodyOriginalIndex);
        }

        public void GetDescription(int handle, out BodyDescription description)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle];
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            GetLane(ref Poses[bundleIndex], innerIndex, out description.Pose);
            GetLane(ref LocalInertias[bundleIndex], innerIndex, out description.LocalInertia);
            GetLane(ref Velocities[bundleIndex], innerIndex, out description.Velocity);
            ref var collidable = ref Collidables[index];
            description.Collidable.Continuity = collidable.Continuity;
            description.Collidable.Shape = collidable.Shape;
            description.Collidable.SpeculativeMargin = collidable.SpeculativeMargin;
        }
        public void SetDescription(int handle, ref BodyDescription description)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle];
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            SetLane(ref Poses[bundleIndex], innerIndex, ref description.Pose);
            SetLane(ref LocalInertias[bundleIndex], innerIndex, ref description.LocalInertia);
            SetLane(ref Velocities[bundleIndex], innerIndex, ref description.Velocity);
            ref var collidable = ref Collidables[index];
            collidable.Continuity = description.Collidable.Continuity;
            collidable.SpeculativeMargin = description.Collidable.SpeculativeMargin;
            //TODO: If users are hitting this with any frequency, supporting the path out of the box would be a good idea.
            //In fact, this function existing at all in the bodies set (where it lacks direct broadphase access) is questionable because of its lack of generality.
            //Something to consider for later.
            Debug.Assert(description.Collidable.Shape.Exists == collidable.Shape.Exists,
                "Setting the description should not be used to add or remove collidables, only to change their shape and properties. " +
                "The broad phase requires notification when a collidable is added or removed. " +
                "You can remove and re-add the body to the simulation, manually notify the broadphase, or create a variant of this function that is broadphase aware.");
            collidable.Shape = description.Collidable.Shape;
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
            var oldCollidableA = Collidables[slotA];
            Collidables[slotA] = Collidables[slotB];
            Collidables[slotB] = oldCollidableA;
            BundleIndexing.GetBundleIndices(slotA, out var bundleA, out var innerA);
            BundleIndexing.GetBundleIndices(slotB, out var bundleB, out var innerB);
            GatherScatter.SwapLanes(ref Poses[bundleA], innerA, ref Poses[bundleB], innerB);
            GatherScatter.SwapLanes(ref Velocities[bundleA], innerA, ref Velocities[bundleB], innerB);
            GatherScatter.SwapLanes(ref LocalInertias[bundleA], innerA, ref LocalInertias[bundleB], innerB);
        }


        /// <summary>
        /// Clears all bodies from the set without returning any memory to the pool.
        /// </summary>
        public unsafe void Clear()
        {
            BodyCount = 0;
            //Empty out all the index-handle mappings.
            Unsafe.InitBlock(HandleToIndex.Memory, 0xFF, (uint)(sizeof(int) * HandleToIndex.Length));
            Unsafe.InitBlock(IndexToHandle.Memory, 0xFF, (uint)(sizeof(int) * IndexToHandle.Length));
            IdPool.Clear();
        }



        public void EnsureCapacity(int bodyCapacity)
        {
            if (IndexToHandle.Length < bodyCapacity)
            {
                InternalResize(bodyCapacity);
            }
            //When ensuring capacity, we assume the user wants to avoid all related resizes.
            //So we bump up the idpool's capacity, too. This is likely a massive overestimate, but it doesn't cost that much, and it does provide the necessary guarantee.
            IdPool.EnsureCapacity(bodyCapacity);
        }
        public void Compact(int bodyCapacity)
        {
            var targetBodyCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(bodyCapacity, BodyCount));
            if (IndexToHandle.Length > targetBodyCapacity)
            {
                InternalResize(targetBodyCapacity);
            }
            IdPool.Compact(bodyCapacity);
        }

        public void Resize(int bodyCapacity)
        {
            var targetBodyCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(bodyCapacity, BodyCount));
            if (IndexToHandle.Length != targetBodyCapacity)
            {
                InternalResize(targetBodyCapacity);
            }
            IdPool.Resize(bodyCapacity);
        }

        /// <summary>
        /// Returns all body resources to the pool used to create them.
        /// </summary>
        /// <remarks>The object can be reused if it is reinitialized by using EnsureCapacity or Resize.</remarks>
        public void Dispose()
        {
            pool.SpecializeFor<BodyPoses>().Return(ref Poses);
            pool.SpecializeFor<BodyVelocities>().Return(ref Velocities);
            pool.SpecializeFor<BodyInertias>().Return(ref LocalInertias);
            pool.SpecializeFor<BodyInertias>().Return(ref Inertias);
            pool.SpecializeFor<int>().Return(ref HandleToIndex);
            pool.SpecializeFor<int>().Return(ref IndexToHandle);
            pool.SpecializeFor<Collidable>().Return(ref Collidables);
            //Zeroing the lengths is useful for reuse- resizes know not to copy old invalid data.
            Poses = new Buffer<BodyPoses>();
            Velocities = new Buffer<BodyVelocities>();
            LocalInertias = new Buffer<BodyInertias>();
            Inertias = new Buffer<BodyInertias>();
            HandleToIndex = new Buffer<int>();
            IndexToHandle = new Buffer<int>();
            IdPool.Dispose();
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
            ref var targetLane = ref GatherScatter.Get(ref targetBundle.InverseInertiaTensor.M11, innerIndex);

            targetLane = inertia.InverseInertiaTensor.M11;
            Unsafe.Add(ref targetLane, Vector<float>.Count) = inertia.InverseInertiaTensor.M21;
            Unsafe.Add(ref targetLane, 2 * Vector<float>.Count) = inertia.InverseInertiaTensor.M22;
            Unsafe.Add(ref targetLane, 3 * Vector<float>.Count) = inertia.InverseInertiaTensor.M31;
            Unsafe.Add(ref targetLane, 4 * Vector<float>.Count) = inertia.InverseInertiaTensor.M32;
            Unsafe.Add(ref targetLane, 5 * Vector<float>.Count) = inertia.InverseInertiaTensor.M33;
            Unsafe.Add(ref targetLane, 6 * Vector<float>.Count) = inertia.InverseMass;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLane(ref BodyInertias targetBundle, int innerIndex, out BodyInertia inertia)
        {
            ref var sourceLane = ref GatherScatter.Get(ref targetBundle.InverseInertiaTensor.M11, innerIndex);

            inertia.InverseInertiaTensor.M11 = sourceLane;
            inertia.InverseInertiaTensor.M21 = Unsafe.Add(ref sourceLane, Vector<float>.Count);
            inertia.InverseInertiaTensor.M22 = Unsafe.Add(ref sourceLane, 2 * Vector<float>.Count);
            inertia.InverseInertiaTensor.M31 = Unsafe.Add(ref sourceLane, 3 * Vector<float>.Count);
            inertia.InverseInertiaTensor.M32 = Unsafe.Add(ref sourceLane, 4 * Vector<float>.Count);
            inertia.InverseInertiaTensor.M33 = Unsafe.Add(ref sourceLane, 5 * Vector<float>.Count);
            inertia.InverseMass = Unsafe.Add(ref sourceLane, 6 * Vector<float>.Count);
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


        //TODO: Filling this out would likely be helpful.
        public void GetPoseByIndex(int index, out BodyPose pose)
        {
            Debug.Assert(index >= 0 && index < BodyCount);
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            GetLane(ref Poses[bundleIndex], innerIndex, out pose);
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

        

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherInertiaForBody(ref float targetInertiaBase, int targetLaneIndex, int bundleIndex, int innerIndex)
        {
            ref var bundleSlot = ref GatherScatter.Get(ref Inertias[bundleIndex].InverseInertiaTensor.M11, innerIndex);
            ref var targetSlot = ref Unsafe.Add(ref targetInertiaBase, targetLaneIndex);
            targetSlot = bundleSlot;
            Unsafe.Add(ref targetSlot, Vector<float>.Count) = Unsafe.Add(ref bundleSlot, Vector<float>.Count);
            Unsafe.Add(ref targetSlot, 2 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 2 * Vector<float>.Count);
            Unsafe.Add(ref targetSlot, 3 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 3 * Vector<float>.Count);
            Unsafe.Add(ref targetSlot, 4 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 4 * Vector<float>.Count);
            Unsafe.Add(ref targetSlot, 5 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 5 * Vector<float>.Count);
            Unsafe.Add(ref targetSlot, 6 * Vector<float>.Count) = Unsafe.Add(ref bundleSlot, 6 * Vector<float>.Count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherPoseForBody(ref float targetPositionBase, ref float targetOrientationBase, int targetLaneIndex, int bundleIndex, int innerIndex)
        {
            ref var sourcePosition = ref GatherScatter.Get(ref Poses[bundleIndex].Position.X, innerIndex);
            ref var sourceOrientation = ref Unsafe.Add(ref sourcePosition, 3 * Vector<float>.Count);
            ref var targetPositionSlot = ref Unsafe.Add(ref targetPositionBase, targetLaneIndex);
            ref var targetOrientationSlot = ref Unsafe.Add(ref targetOrientationBase, targetLaneIndex);
            targetPositionSlot = sourcePosition;
            Unsafe.Add(ref targetPositionSlot, Vector<float>.Count) = Unsafe.Add(ref sourcePosition, Vector<float>.Count);
            Unsafe.Add(ref targetPositionSlot, 2 * Vector<float>.Count) = Unsafe.Add(ref sourcePosition, 2 * Vector<float>.Count);
            targetOrientationSlot = sourceOrientation;
            Unsafe.Add(ref targetOrientationSlot, Vector<float>.Count) = Unsafe.Add(ref sourceOrientation, Vector<float>.Count);
            Unsafe.Add(ref targetOrientationSlot, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceOrientation, 2 * Vector<float>.Count);
            Unsafe.Add(ref targetOrientationSlot, 3 * Vector<float>.Count) = Unsafe.Add(ref sourceOrientation, 3 * Vector<float>.Count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherVelocityForBody(ref float targetLinearVelocityBase, ref float targetAngularVelocityBase, int targetLaneIndex, int bundleIndex, int innerIndex)
        {
            ref var sourceLinear = ref GatherScatter.Get(ref Velocities[bundleIndex].LinearVelocity.X, innerIndex);
            ref var sourceAngular = ref Unsafe.Add(ref sourceLinear, 3 * Vector<float>.Count);
            ref var targetLinearSlot = ref Unsafe.Add(ref targetLinearVelocityBase, targetLaneIndex);
            ref var targetAngularSlot = ref Unsafe.Add(ref targetAngularVelocityBase, targetLaneIndex);
            targetLinearSlot = sourceLinear;
            Unsafe.Add(ref targetLinearSlot, Vector<float>.Count) = Unsafe.Add(ref sourceLinear, Vector<float>.Count);
            Unsafe.Add(ref targetLinearSlot, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceLinear, 2 * Vector<float>.Count);
            targetAngularSlot = sourceAngular;
            Unsafe.Add(ref targetAngularSlot, Vector<float>.Count) = Unsafe.Add(ref sourceAngular, Vector<float>.Count);
            Unsafe.Add(ref targetAngularSlot, 2 * Vector<float>.Count) = Unsafe.Add(ref sourceAngular, 2 * Vector<float>.Count);
        }

        //TODO: In future versions, we will likely store the body position in different forms to allow for extremely large worlds.
        //That will be an opt-in feature. The default implementation will use the FP32 representation, but the user could choose to swap it out for a int64 based representation.
        //This affects other systems- AABB calculation, pose integration, solving, and in extreme (64 bit) cases, the broadphase.
        //We want to insulate other systems from direct knowledge about the implementation of positions when possible.
        //These functions support the solver's needs while hiding absolute positions.
        //In order to support other absolute positions, we'll need alternate implementations of this and other functions.
        //But for the most part, we don't want to pay the overhead of an abstract invocation within the inner loop of the solver. 
        //Given the current limits of C# and the compiler, the best option seems to be a interface implementing struct that provides this functionality.
        //The users would be type specialized by the compiler, avoiding virtual invocation. 
        [MethodImpl(MethodImplOptions.AggressiveInlining)] //Note that this doesn't do anything at the moment- the stackalloc hack blocks inlining.
        internal void GatherInertiaAndPose(ref UnpackedTwoBodyReferences references,
            out Vector3Wide localPositionB, out QuaternionWide orientationA, out QuaternionWide orientationB,
            out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            unsafe
            {
                //Note that we're storing data into vectors by reinterpreting the underlying memory.
                //Unfortunately, there's a compiler bug: https://github.com/dotnet/coreclr/issues/11804
                //Without intervention, a whole lotta stuff goes sideways when the data stored to registers is loaded from memory, 
                //resulting in uninitialized data flying around. Best case, constraints using this diverged massively as the localPositionB was whoknowswhat,
                //worst case the entire process gets torn down by access violations.
                //The inclusion of this hack convinces the JIT to store the gathered values into stack memory rather than aggressively enregistering it.
                //(Why? I didn't bother checking too closely. Maybe the JIT's more aggressive about looking for type punning in the presence of certain kinds of unsafe code.)
                //TODO: remove this once the fix is in. It blocks inlining.
                var hiPleaseDontRemoveThisWithoutTestingOrEverythingMightExplode = stackalloc byte[0];
            }
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);
            ref var targetInertiaBaseB = ref Unsafe.As<Vector<float>, float>(ref inertiaB.InverseInertiaTensor.M11);
            Vector3Wide positionA, positionB;
            ref var targetPositionBaseA = ref Unsafe.As<Vector<float>, float>(ref positionA.X);
            ref var targetPositionBaseB = ref Unsafe.As<Vector<float>, float>(ref positionB.X);
            ref var targetOrientationBaseA = ref Unsafe.As<Vector<float>, float>(ref orientationA.X);
            ref var targetOrientationBaseB = ref Unsafe.As<Vector<float>, float>(ref orientationB.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);

            for (int i = 0; i < references.Count; ++i)
            {
                ref var bundleIndexA = ref Unsafe.Add(ref baseBundleA, i);
                var innerIndexA = Unsafe.Add(ref bundleIndexA, Vector<float>.Count);
                GatherInertiaForBody(ref targetInertiaBaseA, i, bundleIndexA, innerIndexA);
                GatherPoseForBody(ref targetPositionBaseA, ref targetOrientationBaseA, i, bundleIndexA, innerIndexA);
                var bundleIndexB = Unsafe.Add(ref bundleIndexA, 2 * Vector<float>.Count);
                var innerIndexB = Unsafe.Add(ref bundleIndexA, 3 * Vector<float>.Count);
                GatherInertiaForBody(ref targetInertiaBaseB, i, bundleIndexB, innerIndexB);
                GatherPoseForBody(ref targetPositionBaseB, ref targetOrientationBaseB, i, bundleIndexB, innerIndexB);
            }
            Vector3Wide.Subtract(ref positionB, ref positionA, out localPositionB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherInertia(ref UnpackedTwoBodyReferences references,
            out BodyInertias inertiaA, out BodyInertias inertiaB)
        {
            ref var targetInertiaBaseA = ref Unsafe.As<Vector<float>, float>(ref inertiaA.InverseInertiaTensor.M11);
            ref var targetInertiaBaseB = ref Unsafe.As<Vector<float>, float>(ref inertiaB.InverseInertiaTensor.M11);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseBundleA = ref Unsafe.As<Vector<int>, int>(ref references.BundleIndexA);

            for (int i = 0; i < references.Count; ++i)
            {
                ref var bundleIndexA = ref Unsafe.Add(ref baseBundleA, i);
                var innerIndexA = Unsafe.Add(ref bundleIndexA, Vector<float>.Count);
                GatherInertiaForBody(ref targetInertiaBaseA, i, bundleIndexA, innerIndexA);
                var bundleIndexB = Unsafe.Add(ref bundleIndexA, 2 * Vector<float>.Count);
                var innerIndexB = Unsafe.Add(ref bundleIndexA, 3 * Vector<float>.Count);
                GatherInertiaForBody(ref targetInertiaBaseB, i, bundleIndexB, innerIndexB);
            }
        }

        //This looks a little different because it's used by AABB calculation, not constraint pairs.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherDataForBounds(ref int start, int count, out BodyPoses poses, out BodyVelocities velocities, out Vector<int> shapeIndices, out Vector<float> maximumExpansion)
        {
            Debug.Assert(count <= Vector<float>.Count);
            ref var targetPositionBase = ref Unsafe.As<Vector<float>, float>(ref poses.Position.X);
            ref var targetOrientationBase = ref Unsafe.As<Vector<float>, float>(ref poses.Orientation.X);
            ref var targetLinearBase = ref Unsafe.As<Vector<float>, float>(ref velocities.LinearVelocity.X);
            ref var targetAngularBase = ref Unsafe.As<Vector<float>, float>(ref velocities.AngularVelocity.X);
            ref var targetShapeBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            ref var targetExpansionBase = ref Unsafe.As<Vector<float>, float>(ref maximumExpansion);
            for (int i = 0; i < count; ++i)
            {
                var index = Unsafe.Add(ref start, i);
                BundleIndexing.GetBundleIndices(index, out var bundle, out var inner);
                GatherPoseForBody(ref targetPositionBase, ref targetOrientationBase, i, bundle, inner);
                GatherVelocityForBody(ref targetLinearBase, ref targetAngularBase, i, bundle, inner);
                ref var collidable = ref Collidables[index];
                Unsafe.Add(ref targetShapeBase, i) = collidable.Shape.Index;
                //Not entirely pleased with the fact that this pulls in some logic from bounds calculation.
                Unsafe.Add(ref targetExpansionBase, i) = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
            }
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


    }
}
