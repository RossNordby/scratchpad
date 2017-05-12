using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
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

        Optimization nextTarget;

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
            
            var spaceRemaining = solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount - o.BundleIndex;
            if (spaceRemaining <= maximumRegionSizeInBundles)
            {
                ++o.TypeBatchIndex;
                WrapTypeBatch(ref o);
            }
            //Note that the bundle count is not cached; the above type batch may differ.
            o.BundleIndex = Math.Max(0, 
                Math.Min(
                    o.BundleIndex + maximumRegionSizeInBundles / 2, 
                    solver.Batches[o.BatchIndex].TypeBatches[o.TypeBatchIndex].BundleCount - maximumRegionSizeInBundles));

            return o;
        }


        public void Update(int maximumRegionSizeInBundles, int throwaway, BufferPool rawPool, IThreadDispatcher threadDispatcher = null)
        {
            //Note that we require that all regions are bundle aligned. This is important for the typebatch sorting process, which tends to use bulk copies from bundle arrays to cache.
            //If not bundle aligned, those bulk copies would become complex due to the constraint AOSOA layout.
            Debug.Assert((maximumRegionSizeInBundles & 1) == 0, "Region size in bundles should be divisible by two to allow offsets.");
            //No point in optimizing if there are no constraints- this is a necessary test since we assume that 0 is a valid batch index later.
            if (solver.Batches.Count == 0)
                return;

            Optimization target;
            if (shouldOffset)
            {
                //Use the previous frame's start to create the new target.
                target = FindOffsetFrameStart(nextTarget, maximumRegionSizeInBundles);
                Debug.Assert(solver.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex].BundleCount <= maximumRegionSizeInBundles || target.BundleIndex != 0,
                    "On offset frames, the only time a target bundle can be 0 is if the batch is too small for it to be anything else.");
                //Console.WriteLine($"Offset frame targeting {target.BatchIndex}.{target.TypeBatchIndex}:{target.BundleIndex}");
            }
            else
            {
                //Since the constraint set could have changed arbitrarily since the previous execution, validate from batch down.
                target = nextTarget;
                BoundsCheckOldTarget(ref target);
                nextTarget = target;
                nextTarget.BundleIndex += maximumRegionSizeInBundles;
                //Console.WriteLine($"Normal frame targeting {target.BatchIndex}.{target.TypeBatchIndex}:{target.BundleIndex}");
            }
            //Note that we have two separate parallel optimizations over multiple frames. Alternating between them on a per frame basis is a fairly simple way to guarantee
            //eventual convergence in the sort. We only ever push forward the non-offset version; the offset position is based on the nonoffset version's last start position.
            shouldOffset = !shouldOffset;


            var maximumRegionSizeInConstraints = maximumRegionSizeInBundles * Vector<int>.Count;

            var typeBatch = solver.Batches[target.BatchIndex].TypeBatches[target.TypeBatchIndex];
            SortByBodyLocation(typeBatch, target.BundleIndex, Math.Min(typeBatch.ConstraintCount - target.BundleIndex * Vector<int>.Count, maximumRegionSizeInConstraints),
                solver.HandlesToConstraints, bodies.BodyCount, rawPool, threadDispatcher);

        }

        TypeBatch typeBatch;
        IThreadDispatcher threadDispatcher;
        ConstraintLocation[] handlesToConstraints;
        struct Phase1
        {
            public Buffer<int> SortKeys;
            public Buffer<int> SourceIndices;
            public RawBuffer BodyReferencesCache;
            public int SourceStartBundleIndex;
            public int BundlesPerWorker;
            public int BundlesPerWorkerRemainder;
            public int TypeBatchConstraintCount;
        }
        Phase1 phase1;
        void GenerateSortKeys(int workerIndex)
        {
            var localWorkerBundleStart = phase1.BundlesPerWorker * workerIndex + Math.Min(workerIndex, phase1.BundlesPerWorkerRemainder);
            var workerBundleStart = phase1.SourceStartBundleIndex + localWorkerBundleStart;
            var workerBundleCount = workerIndex < phase1.BundlesPerWorkerRemainder ? phase1.BundlesPerWorker + 1 : phase1.BundlesPerWorker;
            var workerConstraintStart = workerBundleStart << BundleIndexing.VectorShift;
            //Note that the number of constraints we can iterate over is clamped by the type batch's constraint count. The last bundle may not be full.
            var workerConstraintCount = Math.Min(phase1.TypeBatchConstraintCount - workerConstraintStart, workerBundleCount << BundleIndexing.VectorShift);
            if (workerConstraintCount <= 0)
                return; //No work remains.
            var localWorkerConstraintStart = localWorkerBundleStart << BundleIndexing.VectorShift;

            typeBatch.GenerateSortKeysAndCopyReferences(
                workerBundleStart, localWorkerBundleStart, workerBundleCount,
                workerConstraintStart, localWorkerConstraintStart, workerConstraintCount,
                ref phase1.SortKeys[localWorkerConstraintStart], ref phase1.SourceIndices[localWorkerConstraintStart], ref phase1.BodyReferencesCache);
        }

        struct Phase2
        {
            public Buffer<int> SortedKeys; //This is only really stored for debug use.
            public Buffer<int> SortedSourceIndices;
            public Buffer<int> ScratchKeys;
            public Buffer<int> ScratchValues;
            public Buffer<int> IndexToHandleCache;
            public RawBuffer PrestepDataCache;
            public RawBuffer AccumulatesImpulsesCache;
            public int KeyUpperBound;
            public int ConstraintsInSortRegionCount;
            //Note that these differ from phase 1- one of the threads is dedicated to a sort. These regard the remaining threads.
            public int BundlesPerWorker;
            public int BundlesPerWorkerRemainder;
        }
        Phase2 phase2;
        void CopyToCacheAndSort(int workerIndex)
        {
            //Sorting only requires that the sort keys and indices be ready. Caching doesn't need to be done yet. 
            //Given that the sort is already very fast and trying to independently multithread it is a bad idea, we'll just bundle it alongside
            //the remaining cache copies. This phase is extremely memory bound and the sort likely won't match the copy duration, but there is no
            //room for complicated schemes at these timescales (<150us). We just try to get as much benefit as we can with a few simple tricks.
            //Most likely we won't get more than about 2.5x speedup on a computer with bandwidth/compute ratios similar to a 3770K with 1600mhz memory.
            if (workerIndex == threadDispatcher.ThreadCount - 1)
            {
                LSBRadixSort.Sort<int, Buffer<int>, Buffer<int>>(
                    ref phase1.SortKeys, ref phase1.SourceIndices,
                    ref phase2.ScratchKeys, ref phase2.ScratchValues, 0, phase2.ConstraintsInSortRegionCount,
                    phase2.KeyUpperBound, threadDispatcher.GetThreadMemoryPool(workerIndex),
                    out phase2.SortedKeys, out phase2.SortedSourceIndices);
            }
            //Note that worker 0 still copies if there's only one thread in the pool. Mainly for debugging purposes.
            if (threadDispatcher.ThreadCount == 1 || workerIndex < threadDispatcher.ThreadCount - 1)
            {
                var localWorkerBundleStart = phase2.BundlesPerWorker * workerIndex + Math.Min(workerIndex, phase2.BundlesPerWorkerRemainder);
                var workerBundleStart = phase1.SourceStartBundleIndex + localWorkerBundleStart;
                var workerBundleCount = workerIndex < phase2.BundlesPerWorkerRemainder ? phase2.BundlesPerWorker + 1 : phase2.BundlesPerWorker;
                var workerConstraintStart = workerBundleStart << BundleIndexing.VectorShift;
                //Note that the number of constraints we can iterate over is clamped by the type batch's constraint count. The last bundle may not be full.
                var workerConstraintCount = Math.Min(phase1.TypeBatchConstraintCount - workerConstraintStart, workerBundleCount << BundleIndexing.VectorShift);
                if (workerConstraintCount <= 0)
                    return; //No work remains.
                var localWorkerConstraintStart = localWorkerBundleStart << BundleIndexing.VectorShift;

                typeBatch.CopyToCache(
                    workerBundleStart, localWorkerBundleStart, workerBundleCount,
                    workerConstraintStart, localWorkerConstraintStart, workerConstraintCount,
                    ref phase2.IndexToHandleCache, ref phase2.PrestepDataCache, ref phase2.AccumulatesImpulsesCache);
            }
        }

        void Regather(int workerIndex)
        {
            var localWorkerBundleStart = phase1.BundlesPerWorker * workerIndex + Math.Min(workerIndex, phase1.BundlesPerWorkerRemainder);
            var workerBundleStart = phase1.SourceStartBundleIndex + localWorkerBundleStart;
            var workerBundleCount = workerIndex < phase1.BundlesPerWorkerRemainder ? phase1.BundlesPerWorker + 1 : phase1.BundlesPerWorker;
            var workerConstraintStart = workerBundleStart << BundleIndexing.VectorShift;
            //Note that the number of constraints we can iterate over is clamped by the type batch's constraint count. The last bundle may not be full.
            var workerConstraintCount = Math.Min(phase1.TypeBatchConstraintCount - workerConstraintStart, workerBundleCount << BundleIndexing.VectorShift);
            if (workerConstraintCount <= 0)
                return; //No work remains.
            var localWorkerConstraintStart = localWorkerBundleStart << BundleIndexing.VectorShift;
            ref var firstSourceIndex = ref phase2.SortedSourceIndices[localWorkerConstraintStart];

            typeBatch.Regather(workerConstraintStart, workerConstraintCount, ref firstSourceIndex, 
                ref phase2.IndexToHandleCache, ref phase1.BodyReferencesCache, ref phase2.PrestepDataCache, ref phase2.AccumulatesImpulsesCache, handlesToConstraints);

        }

       

        void SortByBodyLocation(TypeBatch typeBatch, int bundleStartIndex, int constraintCount, ConstraintLocation[] handlesToConstraints, int bodyCount,
            BufferPool rawPool, IThreadDispatcher threadDispatcher)
        {
            int bundleCount = (constraintCount >> BundleIndexing.VectorShift);
            if ((constraintCount & BundleIndexing.VectorMask) != 0)
                ++bundleCount;

            var intPool = rawPool.SpecializeFor<int>();
            intPool.Take(constraintCount, out phase1.SourceIndices);
            intPool.Take(constraintCount, out phase1.SortKeys);
            intPool.Take(constraintCount, out phase2.ScratchKeys);
            intPool.Take(constraintCount, out phase2.ScratchValues);
            intPool.Take(constraintCount, out phase2.IndexToHandleCache);

            typeBatch.GetBundleTypeSizes(out var bodyReferencesBundleSize, out var prestepBundleSize, out var accumulatedImpulseBundleSize);

            //The typebatch invoked by the worker will cast the body references to the appropriate type. 
            //Using typeless buffers makes it easy to cache the buffers here in the constraint optimizer rather than in the individual type batches.
            rawPool.Take(bundleCount * bodyReferencesBundleSize, out phase1.BodyReferencesCache);
            rawPool.Take(bundleCount * prestepBundleSize, out phase2.PrestepDataCache);
            rawPool.Take(bundleCount * accumulatedImpulseBundleSize, out phase2.AccumulatesImpulsesCache);

            phase1.BundlesPerWorker = bundleCount / threadDispatcher.ThreadCount;
            phase1.BundlesPerWorkerRemainder = bundleCount - phase1.BundlesPerWorker * threadDispatcher.ThreadCount;
            phase1.TypeBatchConstraintCount = typeBatch.ConstraintCount;
            phase1.SourceStartBundleIndex = bundleStartIndex;

            //The second phase uses one worker to sort.
            if (threadDispatcher.ThreadCount > 1)
            {
                phase2.BundlesPerWorker = bundleCount / (threadDispatcher.ThreadCount - 1);
                phase2.BundlesPerWorkerRemainder = bundleCount - phase2.BundlesPerWorker * (threadDispatcher.ThreadCount - 1);
            }
            else
            {
                //We allow one-thread execution for ease of debugging.
                phase2.BundlesPerWorker = bundleCount;
                phase2.BundlesPerWorkerRemainder = 0;
            }
            phase2.ConstraintsInSortRegionCount = constraintCount;
            phase2.KeyUpperBound = bodyCount - 1;

            this.typeBatch = typeBatch;
            this.threadDispatcher = threadDispatcher;
            this.handlesToConstraints = handlesToConstraints;

            threadDispatcher.DispatchWorkers(GenerateSortKeys);
            threadDispatcher.DispatchWorkers(CopyToCacheAndSort);
            threadDispatcher.DispatchWorkers(Regather);

            this.typeBatch = null;
            this.threadDispatcher = null;
            this.handlesToConstraints = null;

            //This is a pure debug function.
            typeBatch.VerifySortRegion(bundleStartIndex, constraintCount, ref phase2.SortedKeys, ref phase2.SortedSourceIndices);

            intPool.Return(ref phase1.SourceIndices);
            intPool.Return(ref phase1.SortKeys);
            intPool.Return(ref phase2.ScratchKeys);
            intPool.Return(ref phase2.ScratchValues);
            intPool.Return(ref phase2.IndexToHandleCache);
            rawPool.Return(ref phase1.BodyReferencesCache);
            rawPool.Return(ref phase2.PrestepDataCache);
            rawPool.Return(ref phase2.AccumulatesImpulsesCache);
        }
    }
}
