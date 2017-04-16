using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading;

namespace SolverPrototype
{
    public partial class Solver
    {
        void InterstageSync(ref int syncStageIndex)
        {
            //No more work is available to claim, but not every thread is necessarily done with the work they claimed. So we need a dedicated sync- upon completing its local work,
            //a worker increments the 'workerCompleted' counter, and the spins on that counter reaching workerCount * stageIndex.
            ++syncStageIndex;
            var neededCompletionCount = context.WorkerCount * syncStageIndex;
            if (Interlocked.Increment(ref context.WorkerCompletedCount) != neededCompletionCount)
            {
                var wait = new SpinWait();
                while (Volatile.Read(ref context.WorkerCompletedCount) < neededCompletionCount)
                {
                    //We know that the wait is going to be short by design. Any call to Thread.Sleep(0) or, much worse, Thread.Sleep(1) would be a terrible mistake-
                    //both will introduce heavy context switches and potentially evict the cache. 
                    //We want to stick primarily to Thread.SpinWaits with the occasional fallback Thread.Yield. Thread.Yield only surrenders control to a thread
                    //that's on the same processor, so we don't have the same risk of evicting the cache.
                    //So, we reset the wait after every yield.
                    //(We could be a little more direct with access to Thread.SpinWait and Thread.Yield, but those aren't public in .NET Standard 1.4.
                    //We could require the user to provide a SpinWait and Yield implementation within the IThreadPool or something, but not every user's target platform will
                    //support direct access either...)

                    //(The SpinWait has a very simple internal structure. You could technically just fiddle with the internal count to force a spin wait of the desired length
                    //or a yield. But we don't really want to take a dependency on the internal memory representation of types that aren't under our control if we can avoid it.)
                    var shouldReset = wait.NextSpinWillYield;
                    wait.SpinOnce();
                    if (shouldReset)
                        wait.Reset();
                }
            }
        }


        void IntermediateWork(int workerIndex)
        {
            int syncStage = 0;
            int blockIndex;
            var endIndex = context.WorkBlocks.Count;
            while ((blockIndex = Interlocked.Increment(ref context.StageIndices[syncStage])) <= endIndex)
            {
                ref var block = ref context.WorkBlocks[blockIndex - 1];
                Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].Prestep(bodies.LocalInertiaBundles, context.Dt, context.InverseDt, block.StartBundle, block.End);
            }

            InterstageSync(ref syncStage);

            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                endIndex = context.BatchBoundaries[batchIndex];
                while ((blockIndex = Interlocked.Increment(ref context.StageIndices[syncStage])) <= endIndex)
                {
                    ref var block = ref context.WorkBlocks[blockIndex - 1];
                    Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].WarmStart(bodies.VelocityBundles, block.StartBundle, block.End);
                }
                InterstageSync(ref syncStage);
            }

            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    endIndex = context.BatchBoundaries[batchIndex];
                    while ((blockIndex = Interlocked.Increment(ref context.StageIndices[syncStage])) <= endIndex)
                    {
                        ref var block = ref context.WorkBlocks[blockIndex - 1];
                        Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].SolveIteration(bodies.VelocityBundles, block.StartBundle, block.End);
                    }
                    InterstageSync(ref syncStage);
                }
            }
        }



        public double IntermediateMultithreadedUpdate(IThreadPool threadPool, BufferPool bufferPool, float dt, float inverseDt)
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

            var stageCount = 1 + Batches.Count * (iterationCount + 1);
            bufferPool.SpecializeFor<int>().Take(stageCount, out context.StageIndices);

            context.StageIndices[0] = 0;
            int stageIndex = 1;
            for (int i = 0; i < iterationCount + 1; ++i)
            {
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    context.StageIndices[stageIndex++] = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                }
            }

            var start = Stopwatch.GetTimestamp();
            threadPool.ForLoop(0, threadPool.ThreadCount, IntermediateWork);
            var end = Stopwatch.GetTimestamp();

            bufferPool.SpecializeFor<int>().Return(ref context.StageIndices);
            context.WorkBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            context.BatchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
            return (end - start) / (double)Stopwatch.Frequency;
        }



    }
}
