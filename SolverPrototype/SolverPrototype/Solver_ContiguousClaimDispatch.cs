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
    public partial class Solver
    {
        interface IStageFunction
        {
            void Execute(ref WorkBlock block);
        }

        struct PrestepStageFunction : IStageFunction
        {
            public float Dt, InverseDt;
            public Solver Solver;
            [MethodImpl(MethodImplOptions.NoInlining)]
            public void Execute(ref WorkBlock block)
            {
                Solver.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].Prestep(Solver.bodies.LocalInertiaBundles, Dt, InverseDt, block.StartBundle, block.End);
            }
        }

        struct WarmStartStageFunction : IStageFunction
        {
            public Solver Solver;
            [MethodImpl(MethodImplOptions.NoInlining)]
            public void Execute(ref WorkBlock block)
            {
                Solver.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].WarmStart(Solver.bodies.VelocityBundles, block.StartBundle, block.End);
            }
        }
        struct SolveStageFunction : IStageFunction
        {
            public Solver Solver;
            [MethodImpl(MethodImplOptions.NoInlining)]
            public void Execute(ref WorkBlock block)
            {
                Solver.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].SolveIteration(Solver.bodies.VelocityBundles, block.StartBundle, block.End);
            }
        }


        private void ContiguousClaimStage<TStageFunction>(ref TStageFunction stageFunction,
            int batchStart, int inclusiveBatchEnd, ref int workerStart, int previousWorkerStart, int nextWorkerStart, ref int syncStage, int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            Debug.Assert(workerStart >= batchStart && workerStart <= inclusiveBatchEnd);
            if (workerStart < batchStart)
                throw new Exception("Bad start index; before batch start.");
            if (workerStart > inclusiveBatchEnd)
                throw new Exception("Bad start index; after batch end.");
            var blockIndex = workerStart;
            //Note that the block index (and later steal index) are barred from progressing beyond the neighbor start locations.
            //This stops a corner case where the worker fails to claim anything before its start is stolen.
            //There are more elegant ways to fix this, but it works.
            while (blockIndex != nextWorkerStart && Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
            {
                if (blockIndex < 0 || blockIndex > inclusiveBatchEnd)
                    throw new Exception("Bad job index.");
                stageFunction.Execute(ref context.WorkBlocks[blockIndex]);
                //Increment and wrap around.
                blockIndex = blockIndex == inclusiveBatchEnd ? batchStart : blockIndex + 1;
            }
            //By now, we've reached the end of the contiguous region in the forward direction. Try walking the other way.
            var stealIndex = workerStart == batchStart ? inclusiveBatchEnd : workerStart - 1;
            while (stealIndex != previousWorkerStart && Interlocked.CompareExchange(ref context.BlockClaims[stealIndex], claimedState, unclaimedState) == unclaimedState)
            {
                //Note that the worker slot is only changed once the steal slot has managed to actually claim something.
                //If you modified the worker start without checking the claim you could end up pushing the worker start into what should be another worker's domain.
                workerStart = stealIndex;
                if (workerStart < 0 || workerStart >= context.WorkBlocks.Count)
                    throw new Exception("Bad work steal index.");
                stageFunction.Execute(ref context.WorkBlocks[workerStart]);
                //Decrement and wrap around.
                stealIndex = workerStart == batchStart ? inclusiveBatchEnd : workerStart - 1;
            }
            Debug.Assert(workerStart >= batchStart && workerStart <= inclusiveBatchEnd);
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
            //The claimed and unclaimed state swap on every stage to allow faster
            int claimedState = 1;
            int unclaimedState = 0;
            //Note that every batch has a different start position. Each covers a different subset of constraints, so they require different start locations.
            //The same concept applies to the prestep- the prestep covers all constraints at once, rather than batch by batch.
            var prestepStage = new PrestepStageFunction { Dt = context.Dt, InverseDt = context.InverseDt, Solver = this };
            ref var previousWorker = ref context.Workers[workerIndex == 0 ? context.WorkerCount - 1 : workerIndex - 1];
            ref var nextWorker = ref context.Workers[workerIndex == context.WorkerCount - 1 ? 0 : workerIndex + 1];
            ContiguousClaimStage(ref prestepStage, 0, context.WorkBlocks.Count - 1,
                ref worker.PrestepStart, previousWorker.PrestepStart, nextWorker.PrestepStart,
                ref syncStage, claimedState, unclaimedState);
            claimedState = 0;
            unclaimedState = 1;

            var warmStartStage = new WarmStartStageFunction { Solver = this };
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                var inclusiveBatchEnd = context.BatchBoundaries[batchIndex] - 1;
                ContiguousClaimStage(ref warmStartStage, batchStart, inclusiveBatchEnd,
                    ref worker.BatchStarts[batchIndex], previousWorker.BatchStarts[batchIndex], nextWorker.BatchStarts[batchIndex],
                    ref syncStage, claimedState, unclaimedState);
            }
            claimedState = 1;
            unclaimedState = 0;

            var solveStage = new SolveStageFunction { Solver = this };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                    var inclusiveBatchEnd = context.BatchBoundaries[batchIndex] - 1;
                    ContiguousClaimStage(ref solveStage, batchStart, inclusiveBatchEnd,
                        ref worker.BatchStarts[batchIndex], previousWorker.BatchStarts[batchIndex], nextWorker.BatchStarts[batchIndex],
                        ref syncStage, claimedState, unclaimedState);
                    InterstageSync(ref syncStage);
                }

                claimedState = claimedState ^ 1;
                unclaimedState = unclaimedState ^ 1;

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
            const int targetBlocksPerBatchPerWorker = 6;
            const int minimumBlockSizeInBundles = 4;

            var maximumBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
            BuildWorkBlocks(bufferPool, minimumBlockSizeInBundles, maximumBlocksPerBatch);
            ValidateWorkBlocks();

            //Create the claims set.
            bufferPool.SpecializeFor<int>().Take(context.WorkBlocks.Count, out context.BlockClaims);
            context.BlockClaims.Clear(0, context.WorkBlocks.Count);



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
            if (workerCount != context.OldWorkerCount)
            {
                //Our old guess for the distribution of blocks won't work. Regenerate an even distribution.
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

                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    CreateUniformDistributionForBatch(batchIndex);
                }
            }
            else
            {
                //Same worker count, so we can generate a new distribution based on the number of blocks each thread completed relative to the old whole.
                //First, figure out how much each worker managed to finish in the previous frame.
                {
                    var previousExclusiveEnd = 0;
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                    {
                        context.Workers[workerIndex].PrestepStart = previousExclusiveEnd;
                        previousExclusiveEnd += (int)(context.WorkerCaches[workerIndex].CompletedPrestepBlocks * context.WorkBlocks.Count);
                    }
                    //Fix up the remainder. Note that it's numerically possible to end up with more remainder than workers due to floating point issues.
                    //Exceptionally rare, but handle it.
                    var remainder = context.WorkBlocks.Count - previousExclusiveEnd;
                    var remainderPerWorker = remainder / workerCount;
                    var remainderRemainder = remainder - remainderPerWorker * workerCount;
                    for (int workerIndex = workerCount - 1; remainder > 1; --workerIndex)
                    {
                        Debug.Assert(workerIndex > 0);
                        var remainderToDistribute = remainderRemainder-- > 0 ? remainderPerWorker + 1 : remainderPerWorker;
                        remainder -= remainderToDistribute;
                        context.Workers[workerIndex].PrestepStart += remainder;
                    }
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                        if (context.Workers[workerIndex].PrestepStart < 0 || context.Workers[workerIndex].PrestepStart >= context.WorkBlocks.Count)
                            throw new Exception("Invalid prestep start.");
                }

                var batchesWithAvailableGuessesCount = Math.Min(Batches.Count, context.OldBatchCount);
                for (int batchIndex = 0; batchIndex < batchesWithAvailableGuessesCount; ++batchIndex)
                {
                    var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                    var batchEnd = context.BatchBoundaries[batchIndex];
                    var batchCount = batchEnd - batchStart;
                    var previousExclusiveEnd = batchStart;
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                    {
                        context.Workers[workerIndex].BatchStarts[batchIndex] = previousExclusiveEnd;
                        previousExclusiveEnd += (int)(context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] * batchCount);
                        Debug.Assert(previousExclusiveEnd <= batchEnd);
                    }
                    //Fix up the remainder. Note that it's numerically possible to end up with more remainder than workers due to floating point issues.
                    //Exceptionally rare, but handle it.
                    var remainder = batchEnd - previousExclusiveEnd;
                    var remainderPerWorker = remainder / workerCount;
                    var remainderRemainder = remainder - remainderPerWorker * workerCount;
                    for (int workerIndex = workerCount - 1; remainder > 1; --workerIndex)
                    {
                        Debug.Assert(workerIndex > 0);
                        var remainderToDistribute = remainderRemainder-- > 0 ? remainderPerWorker + 1 : remainderPerWorker;
                        remainder -= remainderToDistribute;
                        context.Workers[workerIndex].BatchStarts[batchIndex] += remainder;
                    }

                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                        if (context.Workers[workerIndex].BatchStarts[batchIndex] < 0 || context.Workers[workerIndex].BatchStarts[batchIndex] >= context.WorkBlocks.Count)
                            throw new Exception("Invalid batch start.");
                }
                for (int batchIndex = batchesWithAvailableGuessesCount; batchIndex < Batches.Count; ++batchIndex)
                {
                    CreateUniformDistributionForBatch(batchIndex);
                }
            }

            var start = Stopwatch.GetTimestamp();
            threadPool.ForLoop(0, threadPool.ThreadCount, ContiguousClaimWork);
            var end = Stopwatch.GetTimestamp();

            //Update the cached block counts so that we can build a good starting guess for the next frame.
            unsafe
            {
                if (context.WorkerCaches.Memory != null)
                {
                    bufferPool.SpecializeFor<WorkerCache>().Return(ref context.WorkerCaches);
                    for (int i = 0; i < context.OldWorkerCount; ++i)
                        bufferPool.SpecializeFor<float>().Return(ref context.WorkerCaches[i].CompletedBatchBlocks);
                }
            }
            bufferPool.SpecializeFor<WorkerCache>().Take(workerCount, out context.WorkerCaches);

            //It's possible for the number of work blocks to be zero- an empty simulation.
            //Protect against a division by zero.
            if (context.WorkBlocks.Count > 0)
            {
                var inverseBlockCount = 1f / context.WorkBlocks.Count;
                var total = 0f;
                for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                {
                    var nextWorkerIndex = workerIndex == workerCount - 1 ? 0 : workerIndex + 1;
                    var workerStart = context.Workers[workerIndex].PrestepStart;
                    var nextWorkerStart = context.Workers[nextWorkerIndex].PrestepStart;
                    int workerBlocksInPrestep;
                    if (nextWorkerStart > workerStart)
                    {
                        workerBlocksInPrestep = nextWorkerStart - workerStart;
                    }
                    else
                    {
                        workerBlocksInPrestep = nextWorkerStart + context.WorkBlocks.Count - workerStart;
                    }
                    total += context.WorkerCaches[workerIndex].CompletedPrestepBlocks = workerBlocksInPrestep * inverseBlockCount;
                    Debug.Assert(total >= 0 && total <= 1.00001);
                    Debug.Assert(context.WorkerCaches[workerIndex].CompletedPrestepBlocks >= 0 && context.WorkerCaches[workerIndex].CompletedPrestepBlocks <= 1);
                }
                context.OldWorkerCount = workerCount;
            }
            else
            {
                //If there are no blocks, the next frame will still be expecting a guess. 
                //By setting the old worker count to -1, we guarantee that the next frame will just generate a uniform distribution.
                context.OldWorkerCount = -1;
            }

            for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
            {
                bufferPool.SpecializeFor<float>().Take(workerCount, out context.WorkerCaches[workerIndex].CompletedBatchBlocks);
            }

            for (int batchIndex = 0; batchIndex < context.BatchBoundaries.Count; ++batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                var batchEnd = context.BatchBoundaries[batchIndex];
                //There can be no batch without a workblock in the batch, so this division does not require protection.
                Debug.Assert(batchEnd - batchStart > 0);
                float inverseBatchBlockCount = 1f / (batchEnd - batchStart);
                float total = 0;
                for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                {
                    var nextWorkerIndex = workerIndex == workerCount - 1 ? 0 : workerIndex + 1;
                    var workerStart = context.Workers[workerIndex].BatchStarts[batchIndex];
                    var nextWorkerStart = context.Workers[nextWorkerIndex].BatchStarts[batchIndex];
                    Debug.Assert(workerStart >= batchStart && workerStart < batchEnd);
                    Debug.Assert(nextWorkerStart >= batchStart && nextWorkerStart < batchEnd);
                    int workerBlocksInBatch;
                    if (nextWorkerStart > workerStart)
                    {
                        workerBlocksInBatch = nextWorkerStart - workerStart;
                    }
                    else
                    {
                        workerBlocksInBatch = (nextWorkerStart - batchStart) + (batchEnd - workerStart);
                    }
                    total += context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] = workerBlocksInBatch * inverseBatchBlockCount;
                    Debug.WriteLine($"Size: {context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex]}");
                    Debug.Assert(total >= 0 && total <= 1);
                    Debug.Assert(context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] >= 0 && context.WorkerCaches[workerIndex].CompletedBatchBlocks[batchIndex] <= 1);
                }
            }

            //Note that we always just toss the old workers/batch starts sets and start again. This simplifies things at a very, very small cost.
            for (int i = 0; i < workerCount; ++i)
            {
                bufferPool.SpecializeFor<int>().Return(ref context.Workers[i].BatchStarts);
            }
            context.Workers.Dispose(bufferPool.SpecializeFor<Worker>());

            context.WorkBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            context.BatchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
            bufferPool.SpecializeFor<int>().Return(ref context.BlockClaims);
            context.OldBatchCount = Batches.Count;
            return (end - start) / (double)Stopwatch.Frequency;
        }



    }
}
