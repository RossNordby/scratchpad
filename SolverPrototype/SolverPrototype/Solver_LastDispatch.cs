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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void MergeWorkerBounds(ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex)
        {
            for (int i = 0; i < workerIndex; ++i)
            {
                WorkerBounds.MergeIfTouching(ref bounds, ref allWorkerBounds[i]);
            }
            for (int i = workerIndex + 1; i < context.WorkerCount; ++i)
            {
                WorkerBounds.MergeIfTouching(ref bounds, ref allWorkerBounds[i]);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void TraverseForwardUntilBlocked<TStageFunction>(ref TStageFunction stageFunction, int blockIndex, ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex,
            int batchEnd, int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            while (true)
            {
                if (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    bounds.Max = blockIndex + 1; //Exclusive bound.
                    Debug.Assert(blockIndex <= batchEnd);
                    stageFunction.Execute(ref context.WorkBlocks[blockIndex]);
                    //Increment or exit.
                    if (++blockIndex == batchEnd)
                        break;
                }
                else
                {
                    //Already claimed.
                    bounds.Max = blockIndex + 1; //Exclusive bound.
                    break;
                }
            }
            MergeWorkerBounds(ref bounds, ref allWorkerBounds, workerIndex);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void TraverseBackwardUntilBlocked<TStageFunction>(ref TStageFunction stageFunction, int blockIndex, ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex,
            int batchStart, int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            while (true)
            {
                if (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    bounds.Min = blockIndex;
                    Debug.Assert(blockIndex >= batchStart);
                    stageFunction.Execute(ref context.WorkBlocks[blockIndex]);
                    //Decrement or exit.
                    if (blockIndex == batchStart)
                        break;
                    --blockIndex;
                }
                else
                {
                    //Already claimed.
                    bounds.Min = blockIndex;
                    break;
                }
            }
            MergeWorkerBounds(ref bounds, ref allWorkerBounds, workerIndex);
        }
        private void ExecuteStage<TStageFunction>(ref TStageFunction stageFunction, ref Buffer<WorkerBounds> allWorkerBounds, ref Buffer<WorkerBounds> previousWorkerBounds, int workerIndex,
            int batchStart, int batchEnd, ref int workerStart, ref int syncStage,
            int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            //It is possible for a worker to not have any job available in a particular batch. This can only happen when there are more workers than work blocks in the batch.
            //The workers with indices beyond the available work blocks will have their starts all set to -1 by the scheduler.
            //All previous workers will have tightly packed contiguous indices and won't be able to worksteal at all.
            if (workerStart > -1)
            {
                Debug.Assert(workerStart >= batchStart && workerStart < batchEnd);
                var blockIndex = workerStart;

                ref var bounds = ref allWorkerBounds[workerIndex];

                //Just assume the min will be claimed. There's a chance the thread will get preempted or the value will be read before it's actually claimed, but 
                //that's a very small risk and doesn't affect long-term correctness. (It would just somewhat reduce workstealing effectiveness, and so performance.)
                bounds.Min = blockIndex;

                //Note that initialization guarantees a start index in the batch; no test required.
                Debug.Assert(batchStart <= blockIndex && batchEnd > blockIndex);
                TraverseForwardUntilBlocked(ref stageFunction, blockIndex, ref bounds, ref allWorkerBounds, workerIndex, batchEnd, claimedState, unclaimedState);

                //By now, we've reached the end of the contiguous region in the forward direction. Try walking the other way.
                blockIndex = workerStart - 1;
                //Note that there is no guarantee that the block will be in the batch- this could be the leftmost worker.
                if (blockIndex >= batchStart)
                {
                    TraverseBackwardUntilBlocked(ref stageFunction, blockIndex, ref bounds, ref allWorkerBounds, workerIndex, batchStart, claimedState, unclaimedState);
                }

                //All contiguous slots have been claimed. Now just traverse to the end along the right direction.
                while (bounds.Max < batchEnd)
                {
                    TraverseForwardUntilBlocked(ref stageFunction, bounds.Max, ref bounds, ref allWorkerBounds, workerIndex, batchEnd, claimedState, unclaimedState);

                }

                //Traverse backwards.
                while (bounds.Min > batchStart)
                {
                    //Note bounds.Min - 1; Min is inclusive, so in order to access a new location, it must be pushed out.
                    //Note that the above condition uses a > to handle this.
                    TraverseBackwardUntilBlocked(ref stageFunction, bounds.Min - 1, ref bounds, ref allWorkerBounds, workerIndex, batchStart, claimedState, unclaimedState);
                }

                Debug.Assert(bounds.Min == batchStart && bounds.Max == batchEnd);

                //Clear the previous bounds array before the sync so the next stage has fresh data.
                previousWorkerBounds[workerIndex].Min = int.MaxValue;
                previousWorkerBounds[workerIndex].Max = int.MinValue;
            }
            InterstageSync(ref syncStage);
            //Swap the bounds buffers being used before proceeding.
            var tempWorkerBounds = allWorkerBounds;
            allWorkerBounds = previousWorkerBounds;
            previousWorkerBounds = tempWorkerBounds;


        }
        //This works by distributing start indices across the work blocks based on the previous frame's results. Then, during every stage, the workers
        //start at the indices and work along one direction until they reach another claim. At that point they'll revert back to their start position and work the other way.
        //In the next stage, the workers will start at one end of the previously claimed set and work their way across contiguously.
        //This limits the ability of workers to steal jobs from other workers, but very simply guarantees the contiguity of jobs. It also requires very little overhead.
        //Every work block claim test only requires a single interlocked operation. 
        void LastWork(int workerIndex)
        {
            ref var worker = ref context.Workers[workerIndex];
            int syncStage = 0;
            //The claimed and unclaimed state swap after every usage of both pingpong claims buffers.
            int claimedState = 1;
            int unclaimedState = 0;
            var bounds = context.WorkerBoundsA;
            var boundsBackBuffer = context.WorkerBoundsB;
            //Note that every batch has a different start position. Each covers a different subset of constraints, so they require different start locations.
            //The same concept applies to the prestep- the prestep covers all constraints at once, rather than batch by batch.
            var prestepStage = new PrestepStageFunction { Dt = context.Dt, InverseDt = context.InverseDt, Solver = this };
            Debug.Assert(Batches.Count > 0, "Don't dispatch if there are no constraints.");
            ExecuteStage(ref prestepStage, ref bounds, ref boundsBackBuffer, workerIndex, 0, context.WorkBlocks.Count,
                ref worker.PrestepStart, ref syncStage, claimedState, unclaimedState);

            claimedState = 0;
            unclaimedState = 1;
            var warmStartStage = new WarmStartStageFunction { Solver = this };
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                //Don't use the warm start to guess at the solve iteration work distribution.
                var workerBatchStartCopy = worker.BatchStarts[batchIndex];
                ExecuteStage(ref warmStartStage, ref bounds, ref boundsBackBuffer, workerIndex, batchStart, context.BatchBoundaries[batchIndex],
                    ref workerBatchStartCopy, ref syncStage, claimedState, unclaimedState);
            }
            claimedState = 1;
            unclaimedState = 0;

            var solveStage = new SolveStageFunction { Solver = this };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                    ExecuteStage(ref solveStage, ref bounds, ref boundsBackBuffer, workerIndex, batchStart, context.BatchBoundaries[batchIndex],
                        ref worker.BatchStarts[batchIndex], ref syncStage, claimedState, unclaimedState);
                }
                claimedState ^= 1;
                unclaimedState ^= 1;
            }
        }


        public double LastMultithreadedUpdate(IThreadPool threadPool, BufferPool bufferPool, float dt, float inverseDt)
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

            //Note the clear; the block claims must be initialized to 0 so that the first worker stage knows that the data is available to claim.
            bufferPool.SpecializeFor<int>().Take(context.WorkBlocks.Count, out context.BlockClaims);
            context.BlockClaims.Clear(0, context.WorkBlocks.Count);
            bufferPool.SpecializeFor<WorkerBounds>().Take(workerCount, out context.WorkerBoundsA);
            bufferPool.SpecializeFor<WorkerBounds>().Take(workerCount, out context.WorkerBoundsB);



            QuickList<Worker, Buffer<Worker>>.Create(bufferPool.SpecializeFor<Worker>(), workerCount, out context.Workers);
            context.Workers.Count = workerCount;
            for (int i = 0; i < workerCount; ++i)
            {
                bufferPool.SpecializeFor<int>().Take(Batches.Count, out context.Workers[i].BatchStarts);
            }



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
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
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

            var start = Stopwatch.GetTimestamp();
            //While we could be a little more aggressive about culling work with this condition, it doesn't matter much. Have to do it for correctness; worker relies on it.
            if (Batches.Count > 0)
                threadPool.ForLoop(0, threadPool.ThreadCount, LastWork);
            var end = Stopwatch.GetTimestamp();


            //Note that we always just toss the old workers/batch starts sets and start again. This simplifies things at a very, very small cost.
            for (int i = 0; i < workerCount; ++i)
            {
                bufferPool.SpecializeFor<int>().Return(ref context.Workers[i].BatchStarts);
            }
            context.Workers.Dispose(bufferPool.SpecializeFor<Worker>());

            context.WorkBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            context.BatchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
            bufferPool.SpecializeFor<int>().Return(ref context.BlockClaims);
            bufferPool.SpecializeFor<WorkerBounds>().Return(ref context.WorkerBoundsA);
            bufferPool.SpecializeFor<WorkerBounds>().Return(ref context.WorkerBoundsB);
            return (end - start) / (double)Stopwatch.Frequency;
        }



    }
}
