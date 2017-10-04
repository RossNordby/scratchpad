﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
    internal class FreshnessChecker
    {
        int freshnessJobCount;
        int freshnessJobIndex;
        PairCache PairCache;
        ConstraintRemover ConstraintRemover;

        public FreshnessChecker(NarrowPhase narrowPhase)
        {
            PairCache = narrowPhase.PairCache;
            ConstraintRemover = narrowPhase.ConstraintRemover;
        }

        public void CreateJobs(int threadCount, ref QuickList<PreflushJob, Buffer<PreflushJob>> jobs, BufferPool pool)
        {
            if (threadCount > 1)
            {
                const int jobsPerThread = 2; //TODO: Empirical tune; probably just 1.
                freshnessJobCount = Math.Min(threadCount * jobsPerThread, PairCache.Mapping.Count);
                var pairsPerJob = PairCache.Mapping.Count / freshnessJobCount;
                var remainder = PairCache.Mapping.Count - pairsPerJob * freshnessJobCount;
                int previousEnd = 0;
                jobs.EnsureCapacity(jobs.Count + freshnessJobCount, pool.SpecializeFor<PreflushJob>());
                for (int i = 0; i < freshnessJobCount; ++i)
                {
                    ref var job = ref jobs.AllocateUnsafely();
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
            else
            {
                jobs.Add(new PreflushJob { Type = PreflushJobType.CheckFreshness, Start = 0, End = PairCache.Mapping.Count }, pool.SpecializeFor<PreflushJob>());
            }
        }


        public void CheckFreshnessInRegion(int workerIndex, int startIndex, int endIndex)
        {
            var count = endIndex - startIndex;
            var wideCount = count >> 3;
            var remainder = count - (wideCount << 3);
            Debug.Assert((startIndex & 7) == 0 || startIndex == endIndex, "Either this job is empty or the start should be 8 byte aligned for quick reading.");
            //We will check 8 pairs simultaneously. Since the vast majority of pairs are not stale, the ability to skip 8 at a time speeds things up.
            ref var start = ref Unsafe.As<byte, ulong>(ref PairCache.PairFreshness[startIndex]);
            for (int i = 0; i < wideCount; ++i)
            {
                ref var freshnessBatch = ref Unsafe.Add(ref start, i);
                //Perform a binary search for all stale bytes.
                if (freshnessBatch < 0xFFFF_FFFF_FFFF_FFFF)
                {
                    //TODO: Test this against a simple loop.
                    var startOfWide = startIndex + (i << 3);
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
            //Check the remainder of the bytes one by one. Less than 8 left, so no need to be tricky.
            for (int i = endIndex - remainder; i < endIndex; ++i)
            {
                if (PairCache.PairFreshness[i] == 0)
                {
                    EnqueueStaleRemoval(workerIndex, i);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void EnqueueStaleRemoval(int workerIndex, int pairIndex)
        {
            //Note that we have to grab the *old* handle, because the current frame's set of constraint caches do not contain this pair.
            //If they DID contain this pair, then it wouldn't be stale!
            var constraintHandle = PairCache.GetOldConstraintHandle(pairIndex);
            ConstraintRemover.EnqueueRemoval(workerIndex, constraintHandle);
            ref var cache = ref PairCache.NextWorkerCaches[workerIndex];
            cache.PendingRemoves.Add(PairCache.Mapping.Keys[pairIndex], cache.pool.SpecializeFor<CollidablePair>());
        }
    }
}
