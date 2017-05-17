﻿using BEPUutilities2;
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
        
        int manualNaiveBlockIndex;
        int manualNaiveExclusiveEndIndex;
        void ManualNaivePrestep(int workerIndex)
        {
            int blockIndex;
            while ((blockIndex = Interlocked.Increment(ref manualNaiveBlockIndex)) <= manualNaiveExclusiveEndIndex)
            {
                ref var block = ref context.WorkBlocks[blockIndex - 1];
                Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].Prestep(ref bodies.Inertias, context.Dt, context.InverseDt, block.StartBundle, block.End);
            }
        }
        void ManualNaiveWarmStart(int workBlockIndex)
        {
            int blockIndex;
            while ((blockIndex = Interlocked.Increment(ref manualNaiveBlockIndex)) <= manualNaiveExclusiveEndIndex)
            {
                ref var block = ref context.WorkBlocks[blockIndex - 1];
                Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].WarmStart(ref bodies.Velocities, block.StartBundle, block.End);
            }
        }

        void ManualNaiveSolveIteration(int workBlockIndex)
        {
            int blockIndex;
            while ((blockIndex = Interlocked.Increment(ref manualNaiveBlockIndex)) <= manualNaiveExclusiveEndIndex)
            {
                ref var block = ref context.WorkBlocks[blockIndex - 1];
                Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex].SolveIteration(ref bodies.Velocities, block.StartBundle, block.End);
            }
        }




        public double ManualNaiveMultithreadedUpdate(IThreadDispatcher threadPool, BufferPool bufferPool, float dt, float inverseDt)
        {
            var workerCount = context.WorkerCount = threadPool.ThreadCount;
            context.Dt = dt;
            context.InverseDt = inverseDt;
            //First build a set of work blocks.
            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
            //These values are found by empirical tuning. The optimal values may vary by architecture.
            const int targetBlocksPerBatchPerWorker = 4;
            const int minimumBlockSizeInBundles = 4;
            //Note that on a 3770K, the most expensive constraint bundles tend to cost less than 500ns to execute an iteration for. The minimum block size 
            //is trying to balance having pointless numbers of blocks versus the worst case length of worker idling. For example, with a block size of 8,
            //and assuming 500ns per bundle, we risk up to 4 microseconds per iteration-batch worth of idle time.
            //This issue isn't unique to the somewhat odd workstealing scheme we use- it would still be a concern regardless.
            var maximumBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;
            BuildWorkBlocks(bufferPool, minimumBlockSizeInBundles, maximumBlocksPerBatch);
            ValidateWorkBlocks();

            manualNaiveBlockIndex = 0;
            manualNaiveExclusiveEndIndex = context.WorkBlocks.Count;
            var start = Stopwatch.GetTimestamp();
            threadPool.DispatchWorkers(ManualNaivePrestep);

            for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
            {
                manualNaiveBlockIndex = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                manualNaiveExclusiveEndIndex = context.BatchBoundaries[batchIndex];
                threadPool.DispatchWorkers(ManualNaiveWarmStart);
            }

            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int batchIndex = 0; batchIndex < Batches.Count; ++batchIndex)
                {
                    manualNaiveBlockIndex = batchIndex > 0 ? context.BatchBoundaries[batchIndex - 1] : 0;
                    manualNaiveExclusiveEndIndex = context.BatchBoundaries[batchIndex];
                    threadPool.DispatchWorkers(ManualNaiveSolveIteration);
                }
            }

            var end = Stopwatch.GetTimestamp();

            context.WorkBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            context.BatchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
            return (end - start) / (double)Stopwatch.Frequency;
        }



    }
}