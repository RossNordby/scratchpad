using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading;

namespace SolverPrototype
{
    public class ConstraintLayoutOptimizer
    {
        Bodies bodies;
        Solver solver;
        struct Optimization
        {
            /// <summary>
            /// Index of the target constraint bundle to optimize.
            /// </summary>
            public int BundleIndex;
            /// <summary>
            /// Index of the last optimized type batch.
            /// </summary>
            public int TypeBatchIndex;
            /// <summary>
            /// Index of the last optimized batch.
            /// </summary>
            public int BatchIndex;

        }

        Optimization nextTargetWithoutOffset;
        Optimization previousStartWithoutOffset;

        /// <summary>
        /// If true, regions are offset by a half region width. Toggled each frame. Offsets allow the sorted regions to intermix, eventually converging to a full sort.
        /// </summary>
        bool shouldOffset;

        public ConstraintLayoutOptimizer(Bodies bodies, Solver solver)
        {
            this.bodies = bodies;
            this.solver = solver;
        }

        bool WrapBatch(ref Optimization o)
        {
            Debug.Assert(solver.Batches.Count >= 0);
            if (o.BatchIndex >= solver.Batches.Count)
            {
                //Wrap around.
                o = new Optimization();
                return true;
            }
            return false;
        }
        bool WrapTypeBatch(ref Optimization o)
        {
            Debug.Assert(o.BatchIndex <= solver.Batches.Count, "Should only attempt to wrap type batch indices if the batch index is known to be valid.");
            if (o.TypeBatchIndex >= solver.Batches[o.BatchIndex].TypeBatches.Count)
            {
                ++o.BatchIndex;
                if (!WrapBatch(ref o))
                {
                    o.TypeBatchIndex = 0;
                    o.BundleIndex = 0;
                }
                return true;
            }
            return false;
        }

        bool WrapBundle(ref Optimization o)
        {
            Debug.Assert(o.BatchIndex <= solver.Batches.Count && o.TypeBatchIndex <= solver.Batches[o.BatchIndex].TypeBatches.Count,
                "Should only attempt to wrap constraint index if the type batch and batch indices are known to be valid.");
            if (o.BundleIndex >= solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount)
            {
                ++o.TypeBatchIndex;
                if (!WrapTypeBatch(ref o))
                {
                    o.BundleIndex = 0;
                }
                return true;
            }
            return false;
        }
        void BoundsCheckOldTarget(ref Optimization o)
        {
            if (!WrapBatch(ref o))
            {
                if (!WrapTypeBatch(ref o))
                {
                    WrapBundle(ref o);
                }
            }
        }
        Optimization FindOffsetFrameStart(Optimization o, int maximumRegionSizeInBundles)
        {
            BoundsCheckOldTarget(ref o);

            int bundleCount = 0;
            for (int i = 0; i < solver.Batches.Count; ++i)
            {
                var batch = solver.Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    var typeBatch = batch.TypeBatches[j];
                    bundleCount += typeBatch.BundleCount;
                }
            }

            var remainingDistance = bundleCount / 2;
            while (true)
            {
                var typeBatch = solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex];
                var spaceRemaining = typeBatch.BundleCount - o.BundleIndex;
                if (spaceRemaining > remainingDistance)
                {
                    o.BundleIndex += remainingDistance;
                    //Move to the next position >= (x + 0.5) * regionSize, where x is some whole number.
                    var halfRegionSize = maximumRegionSizeInBundles / 2;
                    var halfRegionCount = o.BundleIndex / halfRegionSize;
                    var remainder = o.BundleIndex - halfRegionSize * halfRegionCount;
                    if (remainder > 0)
                        ++halfRegionCount;
                    //In order to be on an offset start, the number of half regions must be odd.
                    halfRegionCount |= 1;
                    var targetBundleIndex = halfRegionCount * halfRegionSize;
                    //Put the bundle index at the target, or as close to it as you can get while staying within the current batch.
                    //If it bumps up against the end, just pull it forward. You don't lose any convergence power here- the region covers all the bundles that
                    //would have been covered by starting at the target, and then some.
                    o.BundleIndex = Math.Max(0, Math.Min(targetBundleIndex, typeBatch.BundleCount - maximumRegionSizeInBundles));
                    return o;
                }
                remainingDistance -= spaceRemaining;
                o.BundleIndex += spaceRemaining;
                WrapBundle(ref o);
            }
        }


        public void Update(int maximumRegionSizeInBundles, int regionCount, BufferPool rawPool, IThreadDispatcher threadDispatcher = null)
        {
            //Note that we require that all regions are bundle aligned. This is important for the typebatch sorting process, which tends to use bulk copies from bundle arrays to cache.
            //If not bundle aligned, those bulk copies would become complex due to the constraint AOSOA layout.
            Debug.Assert((maximumRegionSizeInBundles & 1) == 0, "Region size in bundles should be divisible by two to allow offsets.");
            //No point in optimizing if there are no constraints- this is a necessary test since we assume that 0 is a valid batch index later.
            if (solver.Batches.Count == 0)
                return;

            Optimization nextTarget;
            if (shouldOffset)
            {
                //Use the previous frame's start to create the new target.
                nextTarget = FindOffsetFrameStart(previousStartWithoutOffset, maximumRegionSizeInBundles);
                Debug.Assert(solver.Batches[nextTarget.BatchIndex].TypeBatches[nextTarget.TypeBatchIndex].BundleCount <= maximumRegionSizeInBundles || nextTarget.BundleIndex != 0,
                    "On offset frames, the only time a target bundle can be 0 is if the batch is too small for it to be anything else.");
            }
            else
            {
                //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
                nextTarget = nextTargetWithoutOffset;
                BoundsCheckOldTarget(ref nextTarget);
                previousStartWithoutOffset = nextTargetWithoutOffset;
            }


            //Note that we can know exactly how many targets will exist thanks to the region count, so we can perform unsafe adds.
            var pool = rawPool.SpecializeFor<Optimization>();
            QuickList<Optimization, Buffer<Optimization>>.Create(pool, regionCount, out targets);

            do
            {

                //Add the initial target for optimization. It's already been validated- either by the initial test, or by the previous wrap.
                targets.AddUnsafely(ref nextTarget);
                nextTarget.BundleIndex += maximumRegionSizeInBundles;
                WrapBundle(ref nextTarget);
                //If this is an offset frame, then every target capable of being offset should be. This is required for convergence.
                if (shouldOffset && nextTarget.BundleIndex == 0)
                {
                    nextTarget.BundleIndex = Math.Max(0, Math.Min(
                        nextTarget.BundleIndex + maximumRegionSizeInBundles / 2,
                        solver.Batches[nextTarget.BatchIndex].TypeBatches[nextTarget.TypeBatchIndex].BundleCount - maximumRegionSizeInBundles));
                }
                Debug.Assert(!shouldOffset || solver.Batches[nextTarget.BatchIndex].TypeBatches[nextTarget.TypeBatchIndex].BundleCount <= maximumRegionSizeInBundles || nextTarget.BundleIndex != 0,
                    "On offset frames, the only time a target bundle can be 0 is if the batch is too small for it to be anything else.");

                //If the next target overlaps with the first target, the collection has wrapped around all constraints. Apparently more regions were requested
                //than are available. Stop collection.
                ref var firstTarget = ref targets[0];
                if (nextTarget.BatchIndex == firstTarget.BatchIndex &&
                    nextTarget.TypeBatchIndex == firstTarget.TypeBatchIndex &&
                    nextTarget.BundleIndex < firstTarget.BundleIndex + maximumRegionSizeInBundles)
                {
                    break;
                }
            }
            while (targets.Count < regionCount);

            //Note that we have two separate parallel optimizations over multiple frames. Alternating between them on a per frame basis is a fairly simple way to guarantee
            //eventual convergence in the sort. We only ever push forward the non-offset version; the offset position is based on the nonoffset version's last start position.
            if (!shouldOffset)
            {
                nextTargetWithoutOffset = nextTarget;
            }
            shouldOffset = !shouldOffset;


            var maximumRegionSizeInConstraints = maximumRegionSizeInBundles * Vector<int>.Count;
            //Now that we have a set of scheduled optimizations, execute them. 
            //Note that having the set of optimizations computed up front makes multithreading the loop trivial.
            if (threadDispatcher != null)
            {
                this.threadContexts = threadDispatcher;
                this.maximumRegionSizeInConstraints = maximumRegionSizeInConstraints;
                targetIndex = 0;
                threadDispatcher.DispatchWorkers(HandleWorker);
                this.threadContexts = null;
            }
            else
            {
                for (int i = 0; i < targets.Count; ++i)
                {
                    SortRegion(ref targets[i], rawPool, maximumRegionSizeInConstraints);
                }
            }
            //Give the targets back to the pool.
            targets.Dispose(pool);
        }

        void SortRegion(ref Optimization target, BufferPool pool, int maximumRegionSizeInConstraints)
        {
            var typeBatch = solver.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex];
            typeBatch.SortByBodyLocation(target.BundleIndex, Math.Min(typeBatch.ConstraintCount - target.BundleIndex * Vector<int>.Count, maximumRegionSizeInConstraints), solver.HandlesToConstraints, bodies.BodyCount, pool);

        }

        //Note that this is stored on the heap just to make it easier to pass to the multithreaded workers later. It doesn't truly exist outside of the scope of an update call.
        //This is basically a poor man's closure.
        QuickList<Optimization, Buffer<Optimization>> targets;
        IThreadDispatcher threadContexts;
        int targetIndex;
        int maximumRegionSizeInConstraints;
        void HandleWorker(int workerIndex)
        {
            int incrementedTargetIndex;
            while ((incrementedTargetIndex = Interlocked.Increment(ref targetIndex)) <= targets.Count)
            {
                SortRegion(ref targets[incrementedTargetIndex - 1], threadContexts.GetThreadMemoryPool(workerIndex), maximumRegionSizeInConstraints);
            }
        }
    }
}
