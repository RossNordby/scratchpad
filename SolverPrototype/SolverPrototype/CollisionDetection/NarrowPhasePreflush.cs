using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace SolverPrototype.CollisionDetection
{
    internal enum PreflushJobType
    {

        /// <summary>
        /// Sorts the constraints of a single type across all workers. Used by deterministic preflushes to schedule adds.
        /// Accesses no buffer pools; memory is allocated and returned on main thread.
        /// </summary>
        SortContactConstraintType,
        /// <summary>
        /// Identifies a first guess at the constraint batch to which every new constraint should be added to. 
        /// Accesses no buffer pools; memory is allocated and returned on main thread.
        /// </summary>
        SpeculativeConstraintBatchSearch,
        /// <summary>
        /// Locally sequential addition of every constraint to the associated body lists.
        /// Modifications to constraint graph are independent of the solver changes.
        /// Accesses main thread buffer pool when per-body lists resize.
        /// </summary>
        AddConstraintsToBodyLists,

        //The deterministic constraint add is split into two dispatches. The to-add sort, speculative batch search, and body list add all happen first.
        //The actual addition process occurs afterward alongside the freshness checker.
        //It's slower than the nondeterministic path, but that's the bullet to bite.

        /// <summary>
        /// Adds constraints to the solver in an order determined by the previous sorts and with the help of the speculatively computed batch targets. Locally sequential.
        /// Accesses main thread buffer pool when type batches are created or resize.
        /// </summary>
        DeterministicConstraintAdd,
        /// <summary>
        /// Adds constraints to the solver in an order determined by the collision detection phase. If the collision detection phase is nondeterministic due to threading, then 
        /// this will result in nondeterministic adds to the solver.
        /// Accesses main thread buffer pool when type batches are created or resize.
        /// </summary>
        NondeterministicConstraintAdd,
        /// <summary>
        /// Check the freshness bytes in a region to remove stale pairs.
        /// </summary>
        CheckFreshness,

    }

    [StructLayout(LayoutKind.Explicit)]
    internal struct PreflushJob
    {
        [FieldOffset(0)]
        public PreflushJobType Type;
        /// <summary>
        /// For a deterministic contact constraint sort job, the narrow phase constraint type index to sort.
        /// </summary>
        [FieldOffset(4)]
        public int TypeIndexToSort;
        /// <summary>
        /// Start region of a CheckFreshness or DeterministicConstraintAdd job.
        /// </summary>
        [FieldOffset(4)]
        public int Start;
        /// <summary>
        /// End region of a CheckFreshness or DeterministicConstraintAdd job.
        /// </summary>
        [FieldOffset(8)]
        public int End;
        /// <summary>
        /// Number of worker threads containing constraints to read in the SortContactConstraintType and AddConstraintsToBodyLists phases.
        /// </summary>
        [FieldOffset(8)]
        public int WorkerCount;
        /// <summary>
        /// Start index of a range of constraints stored across the per-worker pending constraint caches. 
        /// Used by NondeterministicConstraintAdd and SpeculativeConstraintBatchSearch.
        /// </summary>
        [FieldOffset(4)]
        public int StartingWorkerIndex;
        /// <summary>
        /// Number of constraints in a range stored across the per-worker pending constraint caches. 
        /// Used by NondeterministicConstraintAdd and SpeculativeConstraintBatchSearch.
        /// </summary>
        [FieldOffset(8)]
        public int ConstraintCount;
    }

    public partial class NarrowPhase<TCallbacks>
    {
        struct SortConstraintTarget
        {
            public int WorkerIndex;
            public int ByteIndexInCache;
            //Note that we cache the handles as we do the initial pass to collect worker indices and byte indices.
            //While this does inflate the memory usage, note that 1024 pending constraints would only cost 16384 bytes. 
            //So extremely large simulations undergoing significant chaos may result in the sorting thread spilling out of L1, but virtually never L2.
            //This caching avoids the need to repeatedly pull in cache lines from the different worker sets. 
            //In chaotic large simulations, the poor cache line utilization (since we're only looking up 4-8 bytes) could result in spilling into L3.
            //As an added bonus, the access pattern during the initial pass is prefetchable, while a comparison-time lookup is not.
            public ulong Handles;
        }
        Buffer<QuickList<SortConstraintTarget, Buffer<SortConstraintTarget>>> sortedConstraints;

        int preflushJobIndex;
        QuickList<PreflushJob, Buffer<PreflushJob>> preflushJobs;
        Action<int> preflushWorkerLoop;
        SpinLock constraintAddLock = new SpinLock();
        void PreflushWorkerLoop(int workerIndex)
        {
            int jobIndex;
            while ((jobIndex = Interlocked.Increment(ref preflushJobIndex)) < preflushJobs.Count)
            {
                ExecutePreflushJob(workerIndex, ref preflushJobs[jobIndex]);
            }
        }

        struct PendingConstraintComparer : IComparerRef<SortConstraintTarget>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe int Compare(ref SortConstraintTarget a, ref SortConstraintTarget b)
            {
                return a.Handles.CompareTo(b.Handles);
            }
        }
        unsafe interface ISortingHandleCollector
        {
            ulong GetHandles(void* memory);
        }

        struct OneBodyHandleCollector : ISortingHandleCollector
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe ulong GetHandles(void* memory)
            {
                //Only one body; can't just return the memory directly.
                return 0UL + *(uint*)memory;
            }
        }
        struct TwoBodyHandleCollector : ISortingHandleCollector
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe ulong GetHandles(void* memory)
            {
                //Two bodies- just reinterpret the existing memory.
                return *(ulong*)memory;
            }
        }


        unsafe void BuildSortingTargets<THandleCollector>(ref QuickList<SortConstraintTarget, Buffer<SortConstraintTarget>> list, int typeIndex, int workerCount) where THandleCollector : struct, ISortingHandleCollector
        {
            var handleCollector = default(THandleCollector);
            for (int i = 0; i < workerCount; ++i)
            {
                ref var workerList = ref overlapWorkers[i].PendingConstraints.pendingConstraintsByType[typeIndex];
                if (workerList.Count > 0)
                {
                    //This is doing redundant integer divides, but we lack type knowledge and it doesn't require any extra memory loading.
                    var entrySizeInBytes = workerList.ByteCount / workerList.Count;
                    int indexInBytes = 0;
                    for (int j = 0; j < workerList.Count; ++j)
                    {
                        ref var constraint = ref list.AllocateUnsafely();
                        constraint.WorkerIndex = i;
                        constraint.ByteIndexInCache = indexInBytes;
                        indexInBytes += entrySizeInBytes;
                        //Note two details:
                        //1) We rely on the layout of memory in the pending constraint add. If the body handles don't occupy the first bytes, this breaks.
                        //2) We rely on the order of body handles. The narrow phase should always guarantee a consistent order.
                        constraint.Handles = handleCollector.GetHandles(workerList.Buffer.Memory + indexInBytes);
                    }
                }
            }
        }

        unsafe void ExecutePreflushJob(int workerIndex, ref PreflushJob job)
        {
            switch (job.Type)
            {
                case PreflushJobType.CheckFreshness:
                    FreshnessChecker.CheckFreshnessInRegion(workerIndex, job.Start, job.End);
                    break;
                case PreflushJobType.SortContactConstraintType:
                    {
                        //The main thread has already allocated lists of appropriate capacities for all types that exist.
                        //We initialize and sort those lists on multiple threads.
                        ref var list = ref sortedConstraints[job.TypeIndexToSort];
                        //One and two body constraints require separate initialization to work in a single pass with a minimum of last second branches.
                        //Use the type index to determine the body count.
                        //This follows the same convention as the GatherOldImpulses and ScatterNewImpulses of the PairCache.
                        //Constraints cover 16 possible cases:
                        //1-4 contacts: 0x3
                        //convex vs nonconvex: 0x4
                        //1 body versus 2 body: 0x8
                        //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
                        if (job.TypeIndexToSort >= 8)
                        {
                            BuildSortingTargets<TwoBodyHandleCollector>(ref list, job.TypeIndexToSort, job.WorkerCount);
                        }
                        else
                        {
                            BuildSortingTargets<OneBodyHandleCollector>(ref list, job.TypeIndexToSort, job.WorkerCount);
                        }
                        
                        //Since duplicates are impossible (as that would imply the narrow phase generated two constraints for one pair), the non-threeway sort is used.
                        PendingConstraintComparer comparer;
                        QuickSort.Sort(ref list.Span[0], 0, list.Count - 1, ref comparer);
                    }
                    break;
                case PreflushJobType.AddConstraintsToBodyLists:
                    {
                        for (int i = 0; i < job.WorkerCount; ++i)
                        {
                            ref var workerList = ref overlapWorkers[i].PendingConstraints.pendingConstraintsByType[job.TypeIndexToSort];
                            if (workerList.Count > 0)
                            {
                                int indexInBytes = 0;
                                for (int j = 0; j < workerList.Count; ++j)
                                {
                                }
                            }
                        }
                    }
                    break;
            }
        }


        protected override void OnPreflush(IThreadDispatcher threadDispatcher)
        {
            //Given the sizes involved, a fixed guess of 128 should be just fine for essentially any simulation. Overkill, but not in a concerning way.
            //Temporarily allocating 1KB of memory isn't a big deal, and we will only touch the necessary subset of it anyway.
            //(There are pathological cases where resizes are still possible, but the constraint remover handles them by not adding unsafely.)
            QuickList<PreflushJob, Buffer<PreflushJob>>.Create(Pool.SpecializeFor<PreflushJob>(), 128, out preflushJobs);
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            FreshnessChecker.CreateJobs(threadCount, ref preflushJobs, Pool);

            //Note: at the moment we use a bit of a hacky scheme where we create jobs unconditionally for the freshness checker, but the constraint adder uses 
            //direct sequential adds when there's only one thread. 

            var start = Stopwatch.GetTimestamp();
            if (threadCount == 1)
            {
                for (int i = 0; i < preflushJobs.Count; ++i)
                {
                    ExecutePreflushJob(0, ref preflushJobs[i]);
                }
                overlapWorkers[0].PendingConstraints.FlushSequentially(Simulation, ref PairCache);
            }
            else
            {
                preflushJobIndex = -1;
                threadDispatcher.DispatchWorkers(preflushWorkerLoop);
            }
            var end = Stopwatch.GetTimestamp();
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i].PendingConstraints.Dispose();
            }
            Console.WriteLine($"Preflush time (us): {1e6 * (end - start) / ((double)Stopwatch.Frequency)}");
            preflushJobs.Dispose(Pool.SpecializeFor<PreflushJob>());
        }
    }
}
