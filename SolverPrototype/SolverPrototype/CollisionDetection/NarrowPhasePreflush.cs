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
        /// Locally sequential addition of every constraint to the associated body lists.
        /// </summary>
        AddConstraintsToBodyLists,

        /// <summary>
        /// Adds a bunch of constraints nondeterministically by locking the batch to claim a batch slot after speculatively identifying a candidate.
        /// </summary>
        NondeterministicAddConstraints,

        /// <summary>
        /// Sorts the constraints of a single type across all workers. Used by deterministic preflushes to schedule adds.
        /// </summary>
        DeterministicSortContactConstraintType,
        DeterministicSpeculativeConstraintBatchSearch,
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
        /// Number of worker threads containing constraints to read in the AddConstraintsToBodyLists phase.
        /// </summary>
        [FieldOffset(4)]
        public int WorkerCount;
    }

    public partial class NarrowPhase<TCallbacks>
    {
        int preflushJobIndex;
        QuickList<PreflushJob, Buffer<PreflushJob>> preflushJobs;
        Action<int> preflushWorkerLoop;
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
                case PreflushJobType.AddConstraintsToBodyLists:
                    for (int i = 0; i < job.WorkerCount; ++i)
                    {
                        ref var cache = ref overlapWorkers[i].PendingConstraints;
                        cache.Flush(Simulation, ref PairCache);
                    }
                    break;
                case PreflushJobType.NondeterministicAddConstraints:
                    break;
            }
        }


        protected override void OnPreflush(IThreadDispatcher threadDispatcher)
        {
            //Given the sizes involved, a fixed guess of 128 should be just fine for essentially any simulation. Overkill, but not in a concerning way.
            //Temporarily allocating 1KB of memory isn't a big deal, and we will only touch the necessary subset of it anyway.
            //(There are pathological cases where resizes are still possible, but the constraint remover handles them by not adding unsafely.)
            QuickList<PreflushJob, Buffer<PreflushJob>>.Create(Pool.SpecializeFor<PreflushJob>(), 128, out preflushJobs);
            OnPreflush(threadDispatcher);
            FreshnessChecker.CreateJobs(threadDispatcher == null ? 1 : threadDispatcher.ThreadCount, ref preflushJobs, Pool);
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
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Preflush time (us): {1e6 * (end - start) / ((double)Stopwatch.Frequency)}");
            FlushPendingConstraintAdds(threadDispatcher);
        }
    }
}
