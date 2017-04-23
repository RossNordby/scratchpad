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
        int TraverseForwardUntilBlocked<TStageFunction>(ref TStageFunction stageFunction, int blockIndex, ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex,
            int batchEnd, int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            //If no claim is made, this defaults to an invalid interval endpoint.
            int highestLocallyClaimedIndex = -1;
            while (true)
            {
                if (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    highestLocallyClaimedIndex = blockIndex;
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
            return highestLocallyClaimedIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int TraverseBackwardUntilBlocked<TStageFunction>(ref TStageFunction stageFunction, int blockIndex, ref WorkerBounds bounds, ref Buffer<WorkerBounds> allWorkerBounds, int workerIndex,
            int batchStart, int claimedState, int unclaimedState)
            where TStageFunction : IStageFunction
        {
            //If no claim is made, this defaults to an invalid interval endpoint.
            int lowestLocallyClaimedIndex = context.WorkBlocks.Count;
            while (true)
            {
                if (Interlocked.CompareExchange(ref context.BlockClaims[blockIndex], claimedState, unclaimedState) == unclaimedState)
                {
                    lowestLocallyClaimedIndex = blockIndex;
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
            return lowestLocallyClaimedIndex;
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
                //Note that we track the largest contiguous region over the course of the stage execution. The batch start of this worker will be set to the 
                //minimum slot of the largest contiguous region so that following iterations will tend to have a better initial work distribution with less work stealing.
                Debug.Assert(batchStart <= blockIndex && batchEnd > blockIndex);
                var highestLocalClaim = TraverseForwardUntilBlocked(ref stageFunction, blockIndex, ref bounds, ref allWorkerBounds, workerIndex, batchEnd, claimedState, unclaimedState);

                //By now, we've reached the end of the contiguous region in the forward direction. Try walking the other way.
                blockIndex = workerStart - 1;
                //Note that there is no guarantee that the block will be in the batch- this could be the leftmost worker.
                int lowestLocalClaim;
                if (blockIndex >= batchStart)
                {
                    lowestLocalClaim = TraverseBackwardUntilBlocked(ref stageFunction, blockIndex, ref bounds, ref allWorkerBounds, workerIndex, batchStart, claimedState, unclaimedState);
                }
                else
                {
                    lowestLocalClaim = batchStart;
                }
                //These are actually two inclusive bounds, so this is count - 1, but as long as we're consistent it's fine.
                //For this first region, we need to check that it's actually a valid region- if the claims were blocked, it might not be.
                var largestContiguousRegionSize = highestLocalClaim - lowestLocalClaim;
                if (largestContiguousRegionSize >= 0)
                    workerStart = lowestLocalClaim;
                else
                    largestContiguousRegionSize = 0; //It was an invalid region, but later invalid regions should be rejected by size. Setting to zero guarantees that later regions have to have at least one open slot.


                //All contiguous slots have been claimed. Now just traverse to the end along the right direction.
                while (bounds.Max < batchEnd)
                {
                    //Each of these iterations may find a contiguous region larger than our previous attempt.
                    lowestLocalClaim = bounds.Max;
                    highestLocalClaim = TraverseForwardUntilBlocked(ref stageFunction, bounds.Max, ref bounds, ref allWorkerBounds, workerIndex, batchEnd, claimedState, unclaimedState);
                    //If the claim at index lowestLocalClaim was blocked, highestLocalClaim will be -1, so the size will be negative.
                    var regionSize = highestLocalClaim - lowestLocalClaim; //again, actually count - 1
                    if (regionSize > largestContiguousRegionSize)
                    {
                        workerStart = lowestLocalClaim;
                        largestContiguousRegionSize = regionSize;
                    }
                }

                //Traverse backwards.
                while (bounds.Min > batchStart)
                {
                    //Note bounds.Min - 1; Min is inclusive, so in order to access a new location, it must be pushed out.
                    //Note that the above condition uses a > to handle this.
                    highestLocalClaim = bounds.Min - 1;
                    lowestLocalClaim = TraverseBackwardUntilBlocked(ref stageFunction, highestLocalClaim, ref bounds, ref allWorkerBounds, workerIndex, batchStart, claimedState, unclaimedState);
                    //If the claim at highestLocalClaim was blocked, lowestLocalClaim will be workblocks.Count, so the size will be negative.
                    var regionSize = highestLocalClaim - lowestLocalClaim; //again, actually count - 1
                    if (regionSize > largestContiguousRegionSize)
                    {
                        workerStart = lowestLocalClaim;
                        largestContiguousRegionSize = regionSize;
                    }
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

        void LastWork(int workerIndex)
        {
            ref var worker = ref context.Workers[workerIndex];
            if (context.WorkBlocks.Count <= context.WorkerCount)
            {
                //Too few blocks to give every worker a job; give the jobs to the first context.WorkBlocks.Count workers.
                worker.PrestepStart = workerIndex < context.WorkBlocks.Count ? workerIndex : -1;
            }
            else
            {
                var blocksPerWorker = context.WorkBlocks.Count / context.WorkerCount;
                var remainder = context.WorkBlocks.Count - blocksPerWorker * context.WorkerCount;
                worker.PrestepStart = blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
            }
            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                var batchStart = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                var batchCount = context.BatchBoundaries[batchIndex] - batchStart;

                if (batchCount <= context.WorkerCount)
                {
                    //Too few blocks to give every worker a job; give the jobs to the first context.WorkBlocks.Count workers.
                    worker.BatchStarts[batchIndex] = workerIndex < batchCount ? batchStart + workerIndex : -1;
                }
                else
                {
                    var blocksPerWorker = batchCount / context.WorkerCount;
                    var remainder = batchCount - blocksPerWorker * context.WorkerCount;
                    worker.BatchStarts[batchIndex] = batchStart + blocksPerWorker * workerIndex + Math.Min(remainder, workerIndex);
                }
            }

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
            //Technically this could mutate prestep starts, but at the moment we rebuild starts every frame anyway so it doesn't matter oen way or the other.
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

            var targetBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
            BuildWorkBlocks(bufferPool, minimumBlockSizeInBundles, targetBlocksPerBatch);
            ValidateWorkBlocks();

            //Note the clear; the block claims must be initialized to 0 so that the first worker stage knows that the data is available to claim.
            bufferPool.SpecializeFor<int>().Take(context.WorkBlocks.Count, out context.BlockClaims);
            context.BlockClaims.Clear(0, context.WorkBlocks.Count);
            bufferPool.SpecializeFor<WorkerBounds>().Take(workerCount, out context.WorkerBoundsA);
            bufferPool.SpecializeFor<WorkerBounds>().Take(workerCount, out context.WorkerBoundsB);
                        
            //Even though the batch starts are filled on the worker thread, we allocate on the main thread since stackallocs require localinit at the moment.
            QuickList<Worker, Buffer<Worker>>.Create(bufferPool.SpecializeFor<Worker>(), workerCount, out context.Workers);
            context.Workers.Count = workerCount;
            for (int i = 0; i < workerCount; ++i)
            {
                bufferPool.SpecializeFor<int>().Take(Batches.Count, out context.Workers[i].BatchStarts);
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
