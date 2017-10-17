﻿using BEPUutilities2;
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
    /// Collection of allocated static collidables.
    /// </summary>
    public class Statics
    {
        //TODO: There is quite a lot of overlap in implementation between this and the bodies collection.
        //Seems like it could be reasonable to bundle common logic.

        /// <summary>
        /// Remaps a static handle to the actual array index of the static.
        /// The backing array index may change in response to cache optimization.
        /// </summary>
        public Buffer<int> HandleToIndex;
        /// <summary>
        /// Remaps a static index to its handle.
        /// </summary>
        public Buffer<int> IndexToHandle;
        /// <summary>
        /// The set of collidables owned by each static. Speculative margins, continuity settings, and shape indices can be changed directly.
        /// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase of the collidable addition or removal.
        /// </summary>
        public Buffer<Collidable> Collidables;

        public Buffer<RigidPose> Poses;
        public IdPool<Buffer<int>> HandlePool;
        protected BufferPool pool;
        public int Count;

        public unsafe Statics(BufferPool pool, int initialCapacity = 4096)
        {
            this.pool = pool;
            InternalResize(initialCapacity);

            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), initialCapacity, out HandlePool);
        }

        unsafe void InternalResize(int targetCapacity)
        {
            Debug.Assert(targetCapacity > 0, "Resize is not meant to be used as Dispose. If you want to return everything to the pool, use Dispose instead.");
            //Note that we base the bundle capacities on the static capacity. This simplifies the conditions on allocation
            targetCapacity = BufferPool<int>.GetLowestContainingElementCount(targetCapacity);
            Debug.Assert(Poses.Length != BufferPool<RigidPoses>.GetLowestContainingElementCount(targetCapacity), "Should not try to use internal resize of the result won't change the size.");
            pool.SpecializeFor<RigidPose>().Resize(ref Poses, targetCapacity, Count);
            pool.SpecializeFor<int>().Resize(ref IndexToHandle, targetCapacity, Count);
            pool.SpecializeFor<int>().Resize(ref HandleToIndex, targetCapacity, Count);
            pool.SpecializeFor<Collidable>().Resize(ref Collidables, targetCapacity, Count);
            //Initialize all the indices beyond the copied region to -1.
            Unsafe.InitBlock(((int*)HandleToIndex.Memory) + Count, 0xFF, (uint)(sizeof(int) * (HandleToIndex.Length - Count)));
            Unsafe.InitBlock(((int*)IndexToHandle.Memory) + Count, 0xFF, (uint)(sizeof(int) * (IndexToHandle.Length - Count)));
            //Collidables beyond the static count should all point to nothing, which corresponds to zero.
            Collidables.Clear(Count, Collidables.Length - Count);
            //Note that we do NOT modify the idpool's internal queue size here. We lazily handle that during adds, and during explicit calls to EnsureCapacity, Compact, and Resize.
            //The idpool's internal queue will often be nowhere near as large as the actual static size except in corner cases, so in the usual case, being lazy saves a little space.
            //If the user wants to guarantee zero resizes, EnsureCapacity provides them the option to do so.
        }

        public unsafe int Add(ref StaticDescription description)
        {
            if (Count == HandleToIndex.Length)
            {
                Debug.Assert(HandleToIndex.Allocated, "The backing memory of the bodies set should be initialized before use. Did you dispose and then not call EnsureCapacity/Resize?");
                //Out of room; need to resize.
                var newSize = HandleToIndex.Length << 1;
                InternalResize(newSize);
            }
            Debug.Assert(Math.Abs(description.Pose.Orientation.Length() - 1) < 1e-6f, "Orientation should be initialized to a unit length quaternion.");
            var handle = HandlePool.Take();
            var index = Count++;
            HandleToIndex[handle] = index;
            IndexToHandle[index] = handle;
            ref var collidable = ref Collidables[index];
            collidable.Shape = description.Collidable.Shape;
            collidable.Continuity = description.Collidable.Continuity;
            collidable.SpeculativeMargin = description.Collidable.SpeculativeMargin;
            //Collidable's broad phase index is left unset. The simulation is responsible for attaching that data.
            Poses[index] = description.Pose;
            return handle;
        }

        /// <summary>
        /// Removes a static from the set by index and returns whether a move occurred. If another static took its place, the move is output.
        /// </summary>
        /// <param name="staticIndex">Index of the static to remove.</param>
        /// <param name="movedStaticOriginalIndex">Original index of the static that was moved into the removed static's slot. -1 if no static had to be moved.</param>
        /// <returns>True if a static was moved, false otherwise.</returns>
        public bool RemoveAt(int staticIndex, out int movedStaticOriginalIndex)
        {
            Debug.Assert(staticIndex >= 0 && staticIndex < Count);
            var handle = IndexToHandle[staticIndex];
            //Move the last static into the removed slot.
            //This does introduce disorder- there may be value in a second overload that preserves order, but it would require large copies.
            //In the event that so many adds and removals are performed at once that they destroy contiguity, it may be better to just
            //explicitly sort after the fact rather than attempt to retain contiguity incrementally. Handle it as a batch, in other words.
            --Count;
            bool staticMoved = staticIndex < Count;
            if (staticMoved)
            {
                movedStaticOriginalIndex = Count;
                //Copy the memory state of the last element down.
                Poses[staticIndex] = Poses[movedStaticOriginalIndex];
                //Note that if you ever treat the world inertias as 'always updated', it would need to be copied here.
                Collidables[staticIndex] = Collidables[movedStaticOriginalIndex];
                //Point the static handles at the new location.
                var lastHandle = IndexToHandle[movedStaticOriginalIndex];
                HandleToIndex[lastHandle] = staticIndex;
                IndexToHandle[staticIndex] = lastHandle;

            }
            else
            {
                movedStaticOriginalIndex = -1;
            }
            //We rely on the collidable references being nonexistent beyond the static count.
            Collidables[Count] = new Collidable();
            //The indices should also be set to all -1's beyond the static count.
            IndexToHandle[Count] = -1;
            HandlePool.Return(handle, pool.SpecializeFor<int>());
            HandleToIndex[handle] = -1;
            return staticMoved;
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
                "This static handle doesn't seem to exist, or the mappings are out of sync. If a handle exists, both directions should match.");
        }

        /// <summary>
        /// Removes a static from the set and returns whether a move occurred. If another static took its place, the move is output.
        /// </summary>
        /// <param name="handle">Handle of the static to remove.</param>
        /// <param name="removedIndex">Former index of the static that was removed.</param>
        /// <param name="movedStaticOriginalIndex">Original index of the static that was moved into the removed static's slot. -1 if no static had to be moved.</param>
        /// <returns>True if a static was moved, false otherwise.</returns>
        public bool Remove(int handle, out int removedIndex, out int movedStaticOriginalIndex)
        {
            ValidateExistingHandle(handle);
            removedIndex = HandleToIndex[handle];
            return RemoveAt(removedIndex, out movedStaticOriginalIndex);
        }

        public void GetDescription(int handle, out StaticDescription description)
        {
            ValidateExistingHandle(handle);
            var index = HandleToIndex[handle];
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            description.Pose = Poses[index];
            ref var collidable = ref Collidables[index];
            description.Collidable.Continuity = collidable.Continuity;
            description.Collidable.Shape = collidable.Shape;
            description.Collidable.SpeculativeMargin = collidable.SpeculativeMargin;
        }
        internal void SetDescriptionByIndex(int index, ref StaticDescription description)
        {
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            Poses[index] = description.Pose;
            ref var collidable = ref Collidables[index];
            collidable.Continuity = description.Collidable.Continuity;
            collidable.SpeculativeMargin = description.Collidable.SpeculativeMargin;
            //Note that we change the shape here. If the collidable transitions from shapeless->shapeful or shapeful->shapeless, the broad phase has to be notified 
            //so that it can create/remove an entry. That's why this function isn't public.
            collidable.Shape = description.Collidable.Shape;
        }


        /// <summary>
        /// Clears all bodies from the set without returning any memory to the pool.
        /// </summary>
        public unsafe void Clear()
        {
            Count = 0;
            //Empty out all the index-handle mappings.
            Unsafe.InitBlock(HandleToIndex.Memory, 0xFF, (uint)(sizeof(int) * HandleToIndex.Length));
            Unsafe.InitBlock(IndexToHandle.Memory, 0xFF, (uint)(sizeof(int) * IndexToHandle.Length));
            HandlePool.Clear();
        }



        public void EnsureCapacity(int capacity)
        {
            if (IndexToHandle.Length < capacity)
            {
                InternalResize(capacity);
            }
            //When ensuring capacity, we assume the user wants to avoid all related resizes.
            //So we bump up the idpool's capacity, too. This is likely a massive overestimate, but it doesn't cost that much, and it does provide the necessary guarantee.
            HandlePool.EnsureCapacity(capacity, pool.SpecializeFor<int>());
        }
        public void Compact(int capacity)
        {
            var targetCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(capacity, Count));
            if (IndexToHandle.Length > targetCapacity)
            {
                InternalResize(targetCapacity);
            }
            HandlePool.Compact(capacity, pool.SpecializeFor<int>());
        }

        public void Resize(int capacity)
        {
            var targetCapacity = BufferPool<int>.GetLowestContainingElementCount(Math.Max(capacity, Count));
            if (IndexToHandle.Length != targetCapacity)
            {
                InternalResize(targetCapacity);
            }
            HandlePool.Resize(capacity, pool.SpecializeFor<int>());
        }

        /// <summary>
        /// Returns all static resources to the pool used to create them.
        /// </summary>
        /// <remarks>The object can be reused if it is reinitialized by using EnsureCapacity or Resize.</remarks>
        public void Dispose()
        {
            pool.SpecializeFor<RigidPose>().Return(ref Poses);
            pool.SpecializeFor<int>().Return(ref HandleToIndex);
            pool.SpecializeFor<int>().Return(ref IndexToHandle);
            pool.SpecializeFor<Collidable>().Return(ref Collidables);
            HandlePool.Dispose(pool.SpecializeFor<int>());
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void GatherPose(ref float targetPositionBase, ref float targetOrientationBase, int targetLaneIndex, int index)
        {
            ref var source = ref Poses[index];
            ref var targetPositionSlot = ref Unsafe.Add(ref targetPositionBase, targetLaneIndex);
            ref var targetOrientationSlot = ref Unsafe.Add(ref targetOrientationBase, targetLaneIndex);
            targetPositionSlot = source.Position.X;
            Unsafe.Add(ref targetPositionSlot, Vector<float>.Count) = source.Position.Y;
            Unsafe.Add(ref targetPositionSlot, 2 * Vector<float>.Count) = source.Position.Z;
            targetOrientationSlot = source.Orientation.X;
            Unsafe.Add(ref targetOrientationSlot, Vector<float>.Count) = source.Orientation.Y;
            Unsafe.Add(ref targetOrientationSlot, 2 * Vector<float>.Count) = source.Orientation.Z;
            Unsafe.Add(ref targetOrientationSlot, 3 * Vector<float>.Count) = source.Orientation.W;
        }

        //This looks a little different because it's used by AABB calculation, not constraint pairs.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void GatherDataForBounds(ref int start, int count, out RigidPoses poses, out Vector<int> shapeIndices, out Vector<float> maximumExpansion)
        {
            Debug.Assert(count <= Vector<float>.Count);
            ref var targetPositionBase = ref Unsafe.As<Vector<float>, float>(ref poses.Position.X);
            ref var targetOrientationBase = ref Unsafe.As<Vector<float>, float>(ref poses.Orientation.X);
            ref var targetShapeBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            ref var targetExpansionBase = ref Unsafe.As<Vector<float>, float>(ref maximumExpansion);
            for (int i = 0; i < count; ++i)
            {
                var index = Unsafe.Add(ref start, i);
                GatherPose(ref targetPositionBase, ref targetOrientationBase, i, index);
                ref var collidable = ref Collidables[index];
                Unsafe.Add(ref targetShapeBase, i) = collidable.Shape.Index;
                //Not entirely pleased with the fact that this pulls in some logic from bounds calculation.
                Unsafe.Add(ref targetExpansionBase, i) = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
            }
        }



    }
}
