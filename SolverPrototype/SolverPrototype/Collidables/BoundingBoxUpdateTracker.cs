using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System.Diagnostics;

namespace SolverPrototype.Collidables
{
    public struct BoundingBoxUpdateTracker
    {
        Buffer<QuickList<int, Buffer<int>>> batchesPerType;

        public BodyCollidables Collidables;
        BufferPool<int> pool;

        /// <summary>
        /// The number of bodies to accumulate per type before executing an AABB update. The more bodies per batch, the less virtual overhead and execution divergence.
        /// However, this should be kept low enough such that the data that has to be gathered by the bounding box update is still usually in L1.
        /// </summary>
        public const int CollidablesPerFlush = 16;

        public BoundingBoxUpdateTracker(BodyCollidables collidables, BufferPool pool)
        {
            this.pool = pool.SpecializeFor<int>();
            //The number of registered types cannot change mid-frame, because adding collidables mid-update is illegal. Can just allocate based on current count.
            pool.SpecializeFor<QuickList<int, Buffer<int>>>().Take(collidables.Shapes.RegisteredTypeCount, out batchesPerType);
            Collidables = collidables;
        }

        public void Add(Bodies bodies, TypedIndex collidableIndex)
        {
            var typeIndex = collidableIndex.Type;
            Debug.Assert(typeIndex >= 0 && typeIndex < batchesPerType.Length, "The preallocated type batch array should be able to hold every type index. Is the type index broken?");
            ref var batchSlot = ref batchesPerType[typeIndex];
            if (!batchSlot.Span.Allocated)
            {
                //No list exists for this type yet.
                QuickList<int, Buffer<int>>.Create(pool, CollidablesPerFlush, out batchSlot);
            }
            batchSlot.AddUnsafely(collidableIndex.Index);
            if (batchSlot.Count == CollidablesPerFlush)
            {
                Collidables[typeIndex].FlushUpdates(bodies, ref batchSlot);
            }
        }

        public void Dispose()
        {
            for (int i = 0; i < batchesPerType.Length; ++i)
            {
                ref var batch = ref batchesPerType[i];
                if (batch.Span.Allocated)
                    batch.Dispose(pool);
            }
            pool.Raw.SpecializeFor<QuickList<int, Buffer<int>>>().Return(ref batchesPerType);
        }

    }
}
