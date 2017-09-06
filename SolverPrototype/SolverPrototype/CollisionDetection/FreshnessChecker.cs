using BEPUutilities2;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;

namespace SolverPrototype.CollisionDetection
{
    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        struct FreshnessJob
        {
            public int Start;
            public int End;
        }
        int freshnessJobCount;
        int freshnessJobIndex;
        Buffer<FreshnessJob> freshnessJobs;

        void CreateFreshnessJobs(IThreadDispatcher threadDispatcher)
        {
            const int jobsPerThread = 4;
            freshnessJobCount = threadDispatcher.ThreadCount * jobsPerThread;
            var pairsPerJob = PairCache.Mapping.Count / freshnessJobCount;
            var remainder = PairCache.Mapping.Count - pairsPerJob * freshnessJobCount;
            Pool.SpecializeFor<FreshnessJob>().Take(freshnessJobCount, out freshnessJobs);
            int previousEnd = 0;
            for (int i = 0; i < freshnessJobCount; ++i)
            {
                ref var job = ref freshnessJobs[i];
                job.Start = previousEnd;
                //The end of every interval except the last one should be aligned on an 8 byte boundary.
                var pairsInJob = i < remainder ? pairsPerJob + 1 : pairsPerJob;
                previousEnd = ((previousEnd + pairsInJob + 7) >> 3) << 3;
                if (previousEnd > PairCache.Mapping.Count)
                    previousEnd = PairCache.Mapping.Count;
                job.End = previousEnd;
            }
            freshnessJobIndex = -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void EnqueueStaleRemoval(int workerIndex, int pairIndex)
        {
            Callbacks.EnqueueConstraintRemoval(workerIndex, PairCache.GetConstraintHandle(pairIndex));
            ref var cache = ref PairCache.NextWorkerCaches[workerIndex];
            cache.PendingRemoves.Add(PairCache.Mapping.Keys[pairIndex], cache.pool.SpecializeFor<CollidablePair>());
        }
        
        void CheckFreshness(int workerIndex)
        {
            int jobIndex;
            while ((jobIndex = Interlocked.Increment(ref freshnessJobIndex)) < freshnessJobCount)
            {
                ref var job = ref freshnessJobs[jobIndex];
                var count = job.End - job.Start;
                var wideCount = count >> 3;
                var remainder = count - (wideCount << 3);
                Debug.Assert((job.Start & 7) == 0 || job.Start == job.End, "Either this job is empty or the start should be 8 byte aligned for quick reading.");
                //We will check 8 pairs simultaneously. Since the vast majority of pairs are not stale, the ability to skip 8 at a time speeds things up.
                ref var start = ref Unsafe.As<byte, ulong>(ref PairCache.PairFreshness[job.Start]);
                for (int i = 0; i < wideCount; ++i)
                {
                    ref var freshnessBatch = ref Unsafe.Add(ref start, i);
                    //Perform a binary search for all stale bytes.
                    if (freshnessBatch < 0xFFFF_FFFF_FFFF_FFFF)
                    {
                        //TODO: Test this against a simple loop.
                        var startOfWide = job.Start + (i << 3);
                        if ((freshnessBatch & 0x0000_0000_FFFF_FFFF) < 0x0000_0000_FFFF_FFFF)
                        {
                            if ((freshnessBatch & 0x0000_0000_0000_FFFF) < 0x0000_0000_0000_FFFF)
                            {
                                if ((freshnessBatch & 0x0000_0000_0000_00FF) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 0);
                                }
                                if ((freshnessBatch & 0x0000_0000_0000_FF00) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 1);
                                }
                            }
                            if ((freshnessBatch & 0x0000_0000_FFFF_0000) < 0x0000_0000_FFFF_0000)
                            {
                                if ((freshnessBatch & 0x0000_0000_00FF_0000) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 2);
                                }
                                if ((freshnessBatch & 0x0000_0000_FF00_0000) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 3);
                                }
                            }
                        }
                        if ((freshnessBatch & 0xFFFF_FFFF_0000_0000) < 0xFFFF_FFFF_0000_0000)
                        {
                            if ((freshnessBatch & 0x0000_FFFF_0000_0000) < 0x0000_FFFF_0000_0000)
                            {
                                if ((freshnessBatch & 0x0000_00FF_0000_0000) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 4);
                                }
                                if ((freshnessBatch & 0x0000_FF00_0000_0000) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 5);
                                }
                            }
                            if ((freshnessBatch & 0xFFFF_0000_0000_0000) < 0xFFFF_0000_0000_0000)
                            {
                                if ((freshnessBatch & 0x00FF_0000_0000_0000) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 6);
                                }
                                if ((freshnessBatch & 0xFF00_0000_0000_0000) == 0)
                                {
                                    EnqueueStaleRemoval(workerIndex, startOfWide + 7);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
