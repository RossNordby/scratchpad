using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace SolverPrototype.CollisionDetection
{
    internal enum PreflushJobType
    {
        /// <summary>
        /// Check the freshness bytes in a region to remove stale pairs.
        /// </summary>
        CheckFreshness,
        /// <summary>
        /// Adds a bunch of constraints nondeterministically by locking when claiming a slot in the solver.
        /// </summary>
        NondeterministicAddConstraints,

        /// <summary>
        /// Sorts the constraints of a single type across all workers. Used by deterministic preflushes to schedule adds.
        /// Accesses no buffer pools; memory is allocated and returned on main thread.
        /// </summary>
        DeterministicSortContactConstraintType,
        /// <summary>
        /// Accesses no buffer pools; memory is allocated and returned on main thread.
        /// </summary>
        DeterministicSpeculativeConstraintBatchSearch,
        /// <summary>
        /// Locally sequential addition of every constraint to the associated body lists.
        /// Modifications to constraint graph are independent of the solver changes.
        /// Accesses main thread buffer pool when per-body lists resize.
        /// </summary>
        DeterministicAddConstraintsToBodyLists,

        //The deterministic constraint add is split into two dispatches. The to-add sort, speculative batch search, and body list add all happen first.
        //The actual addition process occurs afterward alongside the freshness checker.
        //It's slower than the nondeterministic path, but that's the bullet to bite.

        /// <summary>
        /// Adds constraints to the solver in a locally sequential order determined by the previous sorts and with the help of the speculatively computed batch targets.
        /// Accesses main thread buffer pool when type batches are created or resize.
        /// </summary>
        DeterministicConstraintAdd,

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
        /// Start region of the job. Applies to multiple job types.
        /// </summary>
        [FieldOffset(4)]
        public int Start;
        /// <summary>
        /// End region of the job. Applies to multiple job types.
        /// </summary>
        [FieldOffset(8)]
        public int End;
        /// <summary>
        /// Number of worker threads containing constraints to read in the DeterministicAddConstraintsToBodyLists phase.
        /// </summary>
        [FieldOffset(4)]
        public int WorkerCount;
        /// <summary>
        /// Index of the worker being flushed during the NondeterministicAddConstraints phase.
        /// </summary>
        [FieldOffset(4)]
        public int WorkerIndex;
    }

    public partial class NarrowPhase<TCallbacks>
    {
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
        void ExecutePreflushJob(int workerIndex, ref PreflushJob job)
        {
            switch (job.Type)
            {
                case PreflushJobType.CheckFreshness:
                    FreshnessChecker.CheckFreshnessInRegion(workerIndex, job.Start, job.End);
                    break;
                case PreflushJobType.NondeterministicAddConstraints:
                    ref var cache = ref overlapWorkers[job.WorkerIndex].PendingConstraints;
                    cache.FlushNondeterministically(Simulation, ref PairCache, ref constraintAddLock);
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
            for (int i = 0; i < threadCount; ++i)
            {
                preflushJobs.Add(new PreflushJob { Type = PreflushJobType.NondeterministicAddConstraints, WorkerIndex = i }, Pool.SpecializeFor<PreflushJob>());
            }

            var start = Stopwatch.GetTimestamp();
            if (threadDispatcher == null)
            {
                for (int i = 0; i < preflushJobs.Count; ++i)
                {
                    ExecutePreflushJob(0, ref preflushJobs[i]);
                }
            }
            else
            {
                preflushJobIndex = -1;
                threadDispatcher.DispatchWorkers(preflushWorkerLoop);
            }
            for (int i = 0; i < threadCount; ++i)
            {
                overlapWorkers[i].PendingConstraints.Dispose();
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Preflush time (us): {1e6 * (end - start) / ((double)Stopwatch.Frequency)}");
            preflushJobs.Dispose(Pool.SpecializeFor<PreflushJob>());
        }
    }
}
