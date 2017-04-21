using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;

namespace SolverPrototype
{
    //gettin pretty lazy with these dispatch variant names, oh well they're all gonna get deleted
    public partial class Solver
    {
        struct WorkerBounds
        { 
            /// <summary>
            /// Inclusive start of blocks known to be claimed by any worker.
            /// </summary>
            public int Min;
            /// <summary>
            /// Exclusive end of blocks known to be claimed by any worker.
            /// </summary>
            public int Max;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Merge(ref WorkerBounds current, ref WorkerBounds mergeSource)
            {
                if (mergeSource.Min < current.Min)
                    current.Min = mergeSource.Min;
                if (mergeSource.Max > current.Max)
                    current.Max = mergeSource.Max;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool BoundsTouch(ref WorkerBounds a, ref WorkerBounds b)
            {
                //Note that touching is sufficient reason to merge. They don't have to actually intersect.
                return a.Min - b.Max <= 0 && b.Min - a.Max <= 0;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void MergeIfTouching(ref WorkerBounds current, ref WorkerBounds other)
            {
                //Could be a little more clever here if it matters.
                if (BoundsTouch(ref current, ref other))
                    Merge(ref current, ref other);

            }
        }
        void MergeWorkerBounds(ref WorkerBounds bounds, int workerIndex)
        {
            for (int i =0; i < workerIndex; ++i)
            {
                WorkerBounds.MergeIfTouching(ref bounds, ref context.WorkerBounds[i]);
            }
            for (int i = workerIndex + 1; i < context.WorkerCount; ++i)
            {
                WorkerBounds.MergeIfTouching(ref bounds, ref context.WorkerBounds[i]);
            }
        }
        private void ExecuteStage<TStageFunction>(ref TStageFunction stageFunction, int workerIndex,
            int batchStart, int inclusiveBatchEnd, ref int workerStart, ref int syncStage,
            int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            //It is possible for a worker to not have any job available in a particular batch. This can only happen when there are more workers than work blocks in the batch.
            //The workers with indices beyond the available work blocks will have their starts all set to -1 by the scheduler.
            //All previous workers will have tightly packed contiguous indices and won't be able to worksteal at all.
            if (workerStart > -1)
            {
                Debug.Assert(workerStart >= batchStart && workerStart <= inclusiveBatchEnd);
                var blockIndex = workerStart;

                ref var bounds = ref context.WorkerBounds[workerIndex];

                //Just assume the min will be claimed. There's a chance the thread will get preempted or the value will be read before it's actually claimed, but 
                //that's a very small risk and doesn't affect long-term correctness. (It would just somewhat reduce workstealing effectiveness, and so performance.)
                bounds.Min = blockIndex;
                while (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    bounds.Max = blockIndex;
                    Debug.Assert(context.BlockClaims[blockIndex] == claimedState);
                    stageFunction.Execute(ref context.WorkBlocks[blockIndex]);
                    //Increment or exit.
                    if (blockIndex == inclusiveBatchEnd)
                        break;
                    ++blockIndex;
                }
                //Hit a claim; merge touching bounds.
                MergeWorkerBounds(ref bounds, workerIndex);
                //By now, we've reached the end of the contiguous region in the forward direction. Try walking the other way.
                blockIndex = workerStart == batchStart ? inclusiveBatchEnd : workerStart - 1;
                while (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    bounds.Min = blockIndex;
                    Debug.Assert(blockIndex >= batchStart && blockIndex <= inclusiveBatchEnd);
                    stageFunction.Execute(ref context.WorkBlocks[blockIndex]);
                    //Decrement or exit.
                    if (blockIndex == batchStart)
                        break;
                    --blockIndex;
                }
                //Hit a claim; merge touching bounds.
                MergeWorkerBounds(ref bounds, workerIndex);

                //All contiguous slots have been claimed. Now just traverse to the end along the right direction.
                //Every time an existing claim is hit, merge, jump to the far end, and continue.
                var exclusiveEnd = inclusiveBatchEnd + 1;
                while(bounds.Max < exclusiveEnd)
                {
                    blockIndex = bounds.Max;
                    while (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                    {
                        bounds.Max = blockIndex;
                        Debug.Assert(context.BlockClaims[blockIndex] == claimedState);
                        stageFunction.Execute(ref context.WorkBlocks[blockIndex]);               
                        //Increment or exit.
                        if (blockIndex == inclusiveBatchEnd)
                            break;
                        ++blockIndex;
                    }
                    //Hit a claim; merge touching bounds.
                    MergeWorkerBounds(ref bounds, workerIndex);
                }

                //Traverse backwards.

                //TODO: lots of code share possible.


                Debug.Assert(blockIndex >= batchStart && blockIndex <= inclusiveBatchEnd);
            }
            InterstageSync(ref syncStage);
        }
        //This works by distributing start indices across the work blocks based on the previous frame's results. Then, during every stage, the workers
        //start at the indices and work along one direction until they reach another claim. At that point they'll revert back to their start position and work the other way.
        //In the next stage, the workers will start at one end of the previously claimed set and work their way across contiguously.
        //This limits the ability of workers to steal jobs from other workers, but very simply guarantees the contiguity of jobs. It also requires very little overhead.
        //Every work block claim test only requires a single interlocked operation. 
        void ContiguousClaimWork(int workerIndex)
        {
            ref var worker = ref context.Workers[workerIndex];
            int syncStage = 0;
            //The claimed and unclaimed state swap after every usage of both pingpong claims buffers.
            int claimedState = 1;
            int unclaimedState = 0;
            //The claims buffers are just immutable memory pointers, so swapping them doesn't hurt anything.
            var claims = context.BlockClaimsA;
            var claimsBackBuffer = context.BlockClaimsB;
            //Note that every batch has a different start position. Each covers a different subset of constraints, so they require different start locations.
            //The same concept applies to the prestep- the prestep covers all constraints at once, rather than batch by batch.
            var prestepStage = new PrestepStageFunction { Dt = context.Dt, InverseDt = context.InverseDt, Solver = this };
            ref var previousWorker = ref context.Workers[workerIndex == 0 ? context.WorkerCount - 1 : workerIndex - 1];
            ref var nextWorker = ref context.Workers[workerIndex == context.WorkerCount - 1 ? 0 : workerIndex + 1];
            Debug.Assert(Batches.Count > 0, "Don't dispatch if there are no constraints.");
            ContiguousClaimStage(ref prestepStage, 0, context.WorkBlocks.Count - 1,
                ref worker.PrestepStart, worker.BatchStarts[0], ref syncStage,
                ref claims, ref claimsBackBuffer, claimedState, unclaimedState);

            Debug.Assert(claimsBackBuffer[worker.BatchStarts[0]] == 2);

            var warmStartStage = new WarmStartStageFunction { Solver = this };
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                var inclusiveBatchEnd = context.BatchBoundaries[batchIndex] - 1;
                var nextBatchIndex = batchIndex + 1;
                Buffer<int> nextClaimsBuffer;
                if (nextBatchIndex == Batches.Count)
                {
                    nextBatchIndex = 0;
                    //When we reach the last batch, the next batch will be in the next stage- which uses a different claims buffer.
                    nextClaimsBuffer = claims;
                }
                else
                {
                    nextClaimsBuffer = claimsBackBuffer;
                }
                //Don't use the warm start to guess at the solve iteration work distribution.
                var workerBatchStartCopy = worker.BatchStarts[batchIndex];
                ContiguousClaimStage(ref warmStartStage, batchStart, inclusiveBatchEnd,
                    ref workerBatchStartCopy, worker.BatchStarts[nextBatchIndex], ref syncStage,
                    ref claimsBackBuffer, ref nextClaimsBuffer, claimedState, unclaimedState);
            }
            //Two claims buffers have been filled, flip claim states.
            //Note that we didn't explicitly flip the buffers- instead, we just passed them in reversed during the warm start.
            claimedState = 0;
            unclaimedState = 1;
            int claimBufferIndex = 0;

            var solveStage = new SolveStageFunction { Solver = this };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                    var inclusiveBatchEnd = context.BatchBoundaries[batchIndex] - 1;
                    var nextBatchIndex = batchIndex + 1;
                    Buffer<int> nextClaimsBuffer;
                    if (nextBatchIndex == Batches.Count)
                    {
                        nextBatchIndex = 0;
                        //When we reach the last batch, the next batch will be in the next iteration- which uses a different claims buffer.
                        nextClaimsBuffer = claimsBackBuffer;
                    }
                    else
                    {
                        nextClaimsBuffer = claims;
                    }

                    ContiguousClaimStage(ref solveStage, batchStart, inclusiveBatchEnd,
                        ref worker.BatchStarts[batchIndex], worker.BatchStarts[nextBatchIndex], ref syncStage,
                        ref claims, ref nextClaimsBuffer, claimedState, unclaimedState);

                }

                //Swap the claims buffers after every iteration; one whole buffer has been filled.
                var tempClaims = claims;
                claims = claimsBackBuffer;
                claimsBackBuffer = tempClaims;
                if (++claimBufferIndex == 2)
                {
                    claimBufferIndex = 0;
                    //Two claims buffers have been filled, flip claim states.
                    claimedState ^= 1;
                    unclaimedState ^= 1;
                }

            }
        }


        public double ContiguousClaimMultithreadedUpdate(IThreadPool threadPool, BufferPool bufferPool, float dt, float inverseDt)
        {
            var workerCount = context.WorkerCount = threadPool.ThreadCount;
            context.WorkerCompletedCount = 0;
            context.Dt = dt;
            context.InverseDt = inverseDt;
            //First build a set of work blocks.
            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
            //These values are found by empirical tuning. The optimal values may vary by architecture.
            //The goal here is to have just enough blocks that, in the event that we end up some underpowered threads (due to competition or hyperthreading), 
            //there are enough blocks that workstealing will still generally allow the extra threads to be useful.
            const int targetBlocksPerBatchPerWorker = 16;
            const int minimumBlockSizeInBundles = 4;

            var maximumBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
            BuildWorkBlocks(bufferPool, minimumBlockSizeInBundles, maximumBlocksPerBatch);
            ValidateWorkBlocks();

            //Create the claims set. Note that we use two; we'll ping pong between them.
            //This is necessary because every worker 'preclaims' its own start location before the beginning of the next processing stage.
            //When using only one claims buffer, this preclaim could conflict any time the next stage and the current stage overlap 
            //(every prestep->warmstart, any time there is only one constraint batch, or when using flip-flop traversal after every iteration). 
            bufferPool.SpecializeFor<int>().Take(context.WorkBlocks.Count, out context.BlockClaimsA);
            bufferPool.SpecializeFor<int>().Take(context.WorkBlocks.Count, out context.BlockClaimsB);
            context.BlockClaimsA.Clear(0, context.WorkBlocks.Count);
            context.BlockClaimsB.Clear(0, context.WorkBlocks.Count);



            QuickList<Worker, Buffer<Worker>>.Create(bufferPool.SpecializeFor<Worker>(), workerCount, out context.Workers);
            context.Workers.Count = workerCount;
            for (int i = 0; i < workerCount; ++i)
            {
                bufferPool.SpecializeFor<int>().Take(Batches.Count, out context.Workers[i].BatchStarts);
            }

            void CreateUniformDistributionForBatch(int batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                var batchCount = context.BatchBoundaries[batchIndex] - batchStart;

                if (batchCount < workerCount)
                {
                    //Too few blocks to give every worker a job.
                    for (int workerIndex = 0; workerIndex < batchCount; ++workerIndex)
                    {
                        context.Workers[workerIndex].BatchStarts[batchIndex] = batchIndex + workerIndex;
                    }
                    //All remaining workers should just spinwait; -1 start is a code for short circuiting.
                    for (int workerIndex = batchCount; workerIndex < workerCount; ++workerIndex)
                    {
                        context.Workers[workerIndex].BatchStarts[batchIndex] = -1;
                    }
                }
                else
                {
                    var blocksPerWorker = batchCount / workerCount;
                    var remainder = batchCount - blocksPerWorker * workerCount;
                    var previousExclusiveEnd = batchStart;
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                    {
                        Debug.Assert(previousExclusiveEnd >= batchStart && previousExclusiveEnd <= batchStart + batchCount);
                        context.Workers[workerIndex].BatchStarts[batchIndex] = previousExclusiveEnd;
                        previousExclusiveEnd += remainder-- > 0 ? blocksPerWorker + 1 : blocksPerWorker;
                    }
                }
            }
            void CreateUniformDistributionForPrestep()
            {
                if (context.WorkBlocks.Count < workerCount)
                {
                    //Too few blocks to give every worker a job.
                    for (int workerIndex = 0; workerIndex < context.WorkBlocks.Count; ++workerIndex)
                    {
                        context.Workers[workerIndex].PrestepStart = workerIndex;
                    }
                    //All remaining workers should just spinwait; -1 start is a code for short circuiting.
                    for (int workerIndex = context.WorkBlocks.Count; workerIndex < workerCount; ++workerIndex)
                    {
                        context.Workers[workerIndex].PrestepStart = -1;
                    }
                }
                else
                {
                    var previousExclusiveEnd = 0;
                    var blocksPerWorker = context.WorkBlocks.Count / workerCount;
                    var remainder = context.WorkBlocks.Count - blocksPerWorker * workerCount;
                    for (int i = 0; i < workerCount; ++i)
                    {
                        context.Workers[i].PrestepStart = previousExclusiveEnd;
                        previousExclusiveEnd += remainder-- > 0 ? blocksPerWorker + 1 : blocksPerWorker;
                    }
                }
            }
            //if (workerCount != context.OldWorkerCount)
            {
                CreateUniformDistributionForPrestep();
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    CreateUniformDistributionForBatch(batchIndex);
                }
            }
            //else
            //{
            //if (context.OldPrestepCacheInvalid || context.WorkBlocks.Count <= workerCount)
            //{
            //    CreateUniformDistributionForPrestep();
            //}
            //else
            //{
            //    //Generate a new distribution based on the number of blocks each thread completed relative to the old whole.
            //    //First, figure out how much each worker managed to finish in the previous frame.
            //    {
            //        var previousExclusiveEnd = 0;
            //        for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
            //        {
            //            context.Workers[workerIndex].PrestepStart = previousExclusiveEnd;
            //            previousExclusiveEnd += (int)(Math.Max(1, context.WorkerCaches[workerIndex].CompletedPrestepBlocks * context.WorkBlocks.Count));
            //        }
            //        //Fix up the remainder. Note that it's numerically possible to end up with more remainder than workers due to floating point issues.
            //        //Exceptionally rare, but handle it.
            //        var remainder = context.WorkBlocks.Count - previousExclusiveEnd;
            //        if (remainder < 0)
            //        {
            //            //It's possible to end up with a negative remainder- caused by the requirement that no worker has fewer than 1 block.
            //            //In this case, previous workers have to give up jobs.
            //            int pullBack = 1;
            //        }
            //        else
            //        {
            //            var remainderPerWorker = remainder / workerCount;
            //            var remainderRemainder = remainder - remainderPerWorker * workerCount;
            //            for (int workerIndex = workerCount - 1; remainder > 1; --workerIndex)
            //            {
            //                Debug.Assert(workerIndex > 0);
            //                var remainderToDistribute = remainderRemainder-- > 0 ? remainderPerWorker + 1 : remainderPerWorker;
            //                remainder -= remainderToDistribute;
            //                context.Workers[workerIndex].PrestepStart += remainder;
            //            }
            //        }
            //    }
            //}

            //for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            //{
            //    var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
            //    var batchEnd = context.BatchBoundaries[batchIndex];
            //    var batchCount = batchEnd - batchStart;
            //    if (batchIndex >= context.OldBatchCount || context.OldBatchCacheInvalid[batchIndex] || batchCount < workerCount)
            //    {
            //        CreateUniformDistributionForBatch(batchIndex);
            //    }
            //    else
            //    {
            //        var previousExclusiveEnd = batchStart;
            //        var total = 0.0;
            //        for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
            //        {
            //            context.Workers[workerIndex].BatchStarts[batchIndex] = previousExclusiveEnd;
            //            total += context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex];
            //            previousExclusiveEnd += (int)(context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] * batchCount);
            //            Debug.Assert(previousExclusiveEnd <= batchEnd);
            //        }
            //        //Fix up the remainder. Note that it's numerically possible to end up with more remainder than workers due to floating point issues.
            //        //Exceptionally rare, but handle it.
            //        var remainder = batchEnd - previousExclusiveEnd;
            //        var remainderPerWorker = remainder / workerCount;
            //        var remainderRemainder = remainder - remainderPerWorker * workerCount;
            //        for (int workerIndex = workerCount - 1; remainder > 1; --workerIndex)
            //        {
            //            Debug.Assert(workerIndex > 0);
            //            var remainderToDistribute = remainderRemainder-- > 0 ? remainderPerWorker + 1 : remainderPerWorker;
            //            remainder -= remainderToDistribute;
            //            context.Workers[workerIndex].BatchStarts[batchIndex] += remainder;
            //        }
            //    }
            //}
            //}

            //Preclaim the prestep slots in the claims sets using the current prestep starts. 
            for (int i = 0; i < workerCount; ++i)
            {
                //Only preclaim those slots that have workers actually associated with them. Negative starts correspond to cases where not enough work blocks were available.
                if (context.Workers[i].PrestepStart >= 0)
                    context.BlockClaimsA[context.Workers[i].PrestepStart] = 2;
            }

            var start = Stopwatch.GetTimestamp();
            if (Batches.Count > 0) // While we could be a little more aggressive about culling work with this condition, it doesn't matter much. Have to do it for correctness; worker relies on it.
                threadPool.ForLoop(0, threadPool.ThreadCount, ContiguousClaimWork);
            var end = Stopwatch.GetTimestamp();

            //            //Update the cached block counts so that we can build a good starting guess for the next frame.
            //            unsafe
            //            {
            //                if (context.WorkerCaches.Memory != null)
            //                {
            //                    bufferPool.SpecializeFor<WorkerCache>().Return(ref context.WorkerCaches);
            //                    for (int i = 0; i < context.OldWorkerCount; ++i)
            //                        bufferPool.SpecializeFor<float>().Return(ref context.WorkerCaches[i].CompletedBatchBlocks);
            //                }
            //            }
            //            bufferPool.SpecializeFor<WorkerCache>().Take(workerCount, out context.WorkerCaches);

            //            //It's possible for the number of work blocks to be zero- an empty simulation.
            //            //Protect against a division by zero.
            //            if (context.WorkBlocks.Count > 0)
            //            {
            //                var inverseBlockCount = 1f / context.WorkBlocks.Count;
            //                var total = 0f;
            //                for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
            //                {
            //                    var workerStart = context.Workers[workerIndex].PrestepStart;
            //                    if (workerStart == -1)
            //                    {
            //#if DEBUG
            //                        //Worker starts are negative when there were fewer available work blocks in the batch than workers.
            //                        //The first set of workers up to the work block count will each have one block, contiguously stored.
            //                        //All workers beyond the work block count should have workerStart == -1.
            //                        Debug.Assert(context.WorkBlocks.Count < workerCount);
            //                        for (int i = 0; i < workerIndex; ++i)
            //                        {
            //                            Debug.Assert(context.Workers[i].PrestepStart == i);
            //                        }
            //                        for (int i = workerIndex + 1; i < workerCount; ++i)
            //                        {
            //                            Debug.Assert(context.Workers[i].PrestepStart == -1);
            //                        }
            //#endif
            //                        context.WorkerCaches[workerIndex].CompletedPrestepBlocks = 0;

            //                    }
            //                    else
            //                    {
            //                        var nextWorkerIndex = workerIndex == workerCount - 1 ? 0 : workerIndex + 1;
            //                        var nextWorkerStart = context.Workers[nextWorkerIndex].PrestepStart;
            //                        int workerBlocksInPrestep;
            //                        if (nextWorkerStart > workerStart)
            //                        {
            //                            workerBlocksInPrestep = nextWorkerStart - workerStart;
            //                        }
            //                        else
            //                        {
            //                            workerBlocksInPrestep = nextWorkerStart + context.WorkBlocks.Count - workerStart;
            //                        }
            //                        total += context.WorkerCaches[workerIndex].CompletedPrestepBlocks = workerBlocksInPrestep * inverseBlockCount;
            //                        Debug.Assert(total >= 0 && total <= 1.00001);
            //                        Debug.Assert(context.WorkerCaches[workerIndex].CompletedPrestepBlocks >= 0 && context.WorkerCaches[workerIndex].CompletedPrestepBlocks <= 1);
            //                    }
            //                }
            //                context.OldWorkerCount = workerCount;
            //            }
            //            else
            //            {
            //                //If there are no blocks, the next frame will still be expecting a guess. 
            //                //By setting the old worker count to -1, we guarantee that the next frame will just generate a uniform distribution.
            //                context.OldWorkerCount = -1;
            //            }

            //            for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
            //            {
            //                bufferPool.SpecializeFor<float>().Take(workerCount, out context.WorkerCaches[workerIndex].CompletedBatchBlocks);
            //            }

            //            for (int batchIndex = 0; batchIndex < context.BatchBoundaries.Count; ++batchIndex)
            //            {
            //                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
            //                var batchEnd = context.BatchBoundaries[batchIndex];
            //                //There can be no batch without a workblock in the batch, so this division does not require protection.
            //                Debug.Assert(batchEnd - batchStart > 0);
            //                float inverseBatchBlockCount = 1f / (batchEnd - batchStart);
            //                int totalBlocks = 0;
            //                int inversionCount = 0;
            //                for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
            //                {
            //                    var workerStart = context.Workers[workerIndex].BatchStarts[batchIndex];
            //                    if (workerStart == -1)
            //                    {
            //#if DEBUG
            //                        //Worker starts are negative when there were fewer available work blocks in the batch than workers.
            //                        //The first set of workers up to the work block count will each have one block, contiguously stored.
            //                        //All workers beyond the work block count should have workerStart == -1.
            //                        Debug.Assert(batchEnd - batchStart < workerCount);
            //                        for (int i = 0; i < workerIndex; ++i)
            //                        {
            //                            Debug.Assert(context.Workers[i].BatchStarts[batchIndex] == batchStart + i);
            //                        }
            //                        for (int i = workerIndex + 1; i < workerCount; ++i)
            //                        {
            //                            Debug.Assert(context.Workers[i].BatchStarts[batchIndex] == -1);
            //                        }
            //#endif
            //                        context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] = 0;

            //                    }
            //                    else
            //                    {
            //                        var nextWorkerIndex = workerIndex == workerCount - 1 ? 0 : workerIndex + 1;
            //                        var nextWorkerStart = context.Workers[nextWorkerIndex].BatchStarts[batchIndex];
            //                        Debug.Assert(workerStart >= batchStart && workerStart < batchEnd);
            //                        Debug.Assert(nextWorkerStart >= batchStart && nextWorkerStart < batchEnd);
            //                        int workerBlocksInBatch;
            //                        if (nextWorkerStart > workerStart)
            //                        {
            //                            workerBlocksInBatch = nextWorkerStart - workerStart;
            //                        }
            //                        else
            //                        {
            //                            if (++inversionCount > 1)
            //                                throw new Exception($"Inversion count exceeded: {inversionCount}");
            //                            workerBlocksInBatch = (nextWorkerStart - batchStart) + (batchEnd - workerStart);
            //                        }
            //                        totalBlocks += workerBlocksInBatch;
            //                        Debug.Assert(totalBlocks < batchEnd - batchStart);
            //                        context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] = workerBlocksInBatch * inverseBatchBlockCount;
            //                        Debug.Assert(context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] >= 0 && context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] <= 1);
            //                    }
            //                }
            //            }

            //Note that we always just toss the old workers/batch starts sets and start again. This simplifies things at a very, very small cost.
            for (int i = 0; i < workerCount; ++i)
            {
                bufferPool.SpecializeFor<int>().Return(ref context.Workers[i].BatchStarts);
            }
            context.Workers.Dispose(bufferPool.SpecializeFor<Worker>());

            context.WorkBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            context.BatchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
            bufferPool.SpecializeFor<int>().Return(ref context.BlockClaimsA);
            bufferPool.SpecializeFor<int>().Return(ref context.BlockClaimsB);
            context.OldBatchCount = Batches.Count;
            return (end - start) / (double)Stopwatch.Frequency;
        }



    }
}
