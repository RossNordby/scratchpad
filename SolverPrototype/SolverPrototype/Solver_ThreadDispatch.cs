using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
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
        //This is going to look a lot more complicated than would be expected for a series of forloops.
        //A naive implementation would look something like:
        //1) PRESTEP: Parallel dispatch over all constraints, regardless of batch. (Presteps do not write shared data, so there is no need to redispatch per-batch.)
        //2) WARMSTART: Loop over all constraint batches, parallel dispatch over all constraints in batch. (Warmstarts read and write velocities, so batch borders must be respected.)
        //3) SOLVE ITERATIONS: Loop over iterations, loop over all constraint batches, parallel dispatch over all constraints in batch. (Solve iterations also read/write.) 

        //There are a few problems with this approach:
        //1) Fork-join dispatches are not free. Expect ~2us overhead on the main thread for each one, regardless of the workload.
        //If there are 10 constraint batches and 10 iterations, you're up to 1 + 10 + 10 * 10 = 251 dispatches. On the order of half a millisecond in pure overhead.
        //This is just a byproduct of general purpose dispatchers not being able to make use of extremely fine grained application knowledge.
        //Every dispatch has to get the threads rolling and scheduled, then the threads have to figure out when to go back into a blocked state
        //when no more work is unavailable, and so on. Over and over and over again.

        //2) The forloop provider is not guaranteed to maintain a relationship between forloop index and underlying hardware threads across multiple dispatches.
        //In fact, we should expect the opposite. Work stealing is an important feature for threadpools to avoid pointless idle time.
        //Unfortunately, this can destroy potential cache locality across dispatches. Instead of having the same core with the data warm in its L1 and L2 caches from the previous
        //solver iteration, it'll end up working on some other region that was previously owned by a different core. 
        //This can actually get pretty bad- consider a multiprocessor system where each processor has its own cache. An oblivious thread pool could schedule a region
        //on a different processor. At that point, there's no good outcome.
        //But you don't have to resort to big servers to see something like this- some processors, notably the recent Ryzen line, actually behave a bit like 
        //multiple processors that happen to be stuck on the same chip. If the application requires tons of intercore communication, performance will suffer.
        //And of course, cache misses just suck.

        //3) Work stealing implementations that lack application knowledge will tend to make a given worker operate across noncontiguous regions, harming locality.

        //So what do we do? We have special guarantees:
        //1) We have to do a bunch of solver iterations in sequence, covering the exact same data over and over. Even the prestep and warmstart cover a lot of the same data.
        //2) We can control the dispatch sizes within a frame. They're going to be the same, over and over, and the next dispatch follows immediately after the last.
        //3) We can guarantee that individual work blocks are fairly small. (A handful of microseconds.)

        //So, there's a few parts to the solution as implemented:
        //1) Dispatch *once* and perform fine grained synchronization with busy waits to block at constraint batch borders. Unless the operating system
        //reschedules a thread (which is very possible, but not a constant occurrence), a worker index will stay associated with the same underlying hardware.
        //2) Work blocks are associated with workers by index in a prepass. Once a worker has exhausted their preallocated blocks, they can worksteal another unhandled block.
        //Critically, by stealing it, that worker takes ownership of the region in future iterations.
        //3) In a prepass, sort the previous frame's workblock-worker mapping so that each worker's initial responsibility is contiguous. This may result in 
        //more workstealing in the near term, though- there is no guarantee that the expanded contiguous region costs the same amount as the previous region. 
        //However, over multiple iterations and frames, it will tend to mostly converge. This is a relatively low impact feature, but it's basically free.

        //So, for the most part, the same core will work on the same data across the solve. Hooray!

        //A couple of notes:
        //1) We explicitly don't care about maintaining worker-data relationships between frames. The cache will likely be trashed by the rest of the program- even other parts
        //of the physics simulation will evict stuff. The persistent worker-block mapping is there solely to provide a better initial guess for work scheduling.
        //Think of it as profiling driven load balancing.
        //2) The above 'solution' glossed over prestep and warmstart a bit. Prestep doesn't have to worry about batch boundaries, and warmstart- while operating on the same data-
        //doesn't have the same performance characteristics as a full solve iteration and so can't be used as a reliable profiling data source. In other words,
        //neither the prestep nor warmstart can *modify* the work block allocation usefully, but ideally they still *use* it initially, but also perform work stealing some other way.


        internal struct WorkBlock
        {
            public int BatchIndex;
            public int TypeBatchIndex;
            /// <summary>
            /// Index of the first bundle in the block.
            /// </summary>
            public int StartBundle;
            /// <summary>
            /// Exlusive end index of the bundle. Index of the last bundle in the block is End - 1.
            /// </summary>
            public int End;
        }


        internal struct Worker
        {
            //generics yay
            public QuickList<QuickList<int, Buffer<int>>, Buffer<QuickList<int, Buffer<int>>>> BlocksOwnedInBatches;
        }
        /// <summary>
        /// Creates a set of work blocks for worker threads to operate on and steal. 
        /// </summary>
        /// <param name="workerCount">Number of thread workers available.</param>
        /// <param name="solver">Solver doing the solve.</param>
        /// <param name="bufferPool">Pool to pull temporary resources from.</param>
        /// <param name="blockClaims">Preallocated claim flags for individual blocks.</param>
        /// <param name="workBlocks">Set of bundle regions for threads to execute.</param>
        /// <param name="batchBoundaries">Exclusive end indices of each batch's work block region.</param>
        static void CreateWorkBlocks(int workerCount, Solver solver, BufferPool bufferPool,
             ref QuickList<Worker, Buffer<Worker>> workerContexts,
             out QuickList<WorkBlock, Buffer<WorkBlock>> workBlocks,
             out QuickList<int, Buffer<int>> blockClaims,
             out QuickList<int, Buffer<int>> batchBoundaries)
        {
            //The block size should be relatively small to give the workstealer something to do, but we don't want to go crazy with the number of blocks.
            //These values are found by empirical tuning. The optimal values may vary by architecture.
            const int targetBlocksPerBatchPerWorker = 16;
            const int minimumBlockSizeInBundles = 8;
            //Note that on a 3770K, the most expensive constraint bundles tend to cost less than 500ns to execute an iteration for. The minimum block size 
            //is trying to balance having pointless numbers of blocks versus the worst case length of worker idling. For example, with a block size of 8,
            //and assuming 500ns per bundle, we risk up to 4 microseconds per iteration-batch worth of idle time.
            //This issue isn't unique to the somewhat odd workstealing scheme we use- it would still be a concern regardless.

            var maximumBlocksPerBatch = workerCount * targetBlocksPerBatchPerWorker;

            QuickList<WorkBlock, Buffer<WorkBlock>>.Create(bufferPool.SpecializeFor<WorkBlock>(), maximumBlocksPerBatch * solver.Batches.Count, out workBlocks);
            QuickList<int, Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), solver.Batches.Count, out batchBoundaries);
            for (int batchIndex = 0; batchIndex < solver.Batches.Count; ++batchIndex)
            {
                var batch = solver.Batches[batchIndex];
                var bundleCount = 0;
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    bundleCount += batch.TypeBatches[typeBatchIndex].BundleCount;
                }
                //Create a goal size for the blocks based on the number of bundles present.
                var targetBlockSizeInBundles = bundleCount / maximumBlocksPerBatch;
                if (bundleCount - targetBlockSizeInBundles * maximumBlocksPerBatch > 0)
                    ++targetBlockSizeInBundles;
                if (targetBlockSizeInBundles < minimumBlockSizeInBundles)
                    targetBlockSizeInBundles = minimumBlockSizeInBundles;

                //Walk through the type batches in order. Avoid tiny 'remainder' batches by spreading any remainder over all previous batches.
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    var typeBatchBlockCount = typeBatch.BundleCount / targetBlockSizeInBundles;
                    var remainder = typeBatch.BundleCount - typeBatchBlockCount * targetBlockSizeInBundles;
                    if (remainder > 0)
                        ++typeBatchBlockCount;
                    WorkBlock block;
                    block.BatchIndex = batchIndex;
                    block.TypeBatchIndex = typeBatchIndex;
                    block.End = 0;
                    for (int i = 0; i < typeBatchBlockCount; ++i)
                    {
                        int blockBundleCount = remainder-- > 0 ? targetBlockSizeInBundles + 1 : targetBlockSizeInBundles;
                        //Use the previous end as the new start.
                        block.StartBundle = block.End;
                        block.End = block.StartBundle + blockBundleCount;
                        Debug.Assert(block.StartBundle >= 0 && block.StartBundle < typeBatch.BundleCount);
                        Debug.Assert(block.End >= block.StartBundle + minimumBlockSizeInBundles && block.End <= typeBatch.BundleCount);
                        workBlocks.AddUnsafely(ref block);
                    }
                }
                batchBoundaries.AddUnsafely(ref workBlocks.Count);
            }

            //Create the claims set.
            QuickList<int, Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), workBlocks.Count, out blockClaims);
            blockClaims.Span.Clear(0, workBlocks.Count);
            blockClaims.Count = workBlocks.Count;

            //Now that we have a set of work blocks updated for the current state of the simulation, revalidate the ownership across workers.
            //We have three goals:
            //1) Make ownership regions contiguous to improve spatial locality.
            //2) Adapt to changes in the number of workers.
            //3) Adapt to changes in work blocks.
            //We completely ignore the previous state of work blocks- the only thing we care about is how much each individual worker was contributing to the final result.
            //This provides a guess about how expensiv regions are. By forcing contiguity of regions, we may imbalance the load temporarily. 
            //Workstealing will address any induced load imbalance in the short term, and over multiple frames, the initial guess should tend to converge
            //to something reasonable that minimizes the number of required steals.


            //Any contexts that exist and will continue to exist should have their batch capacities checked.    
            //Note that changes in batch count do not destroy our guesses from the previous frame; the only batches that are ever removed are those which are at the very end of the list.
            //In other words, removes don't change order. (And adds just append, so they're fine too.)
            //So, for all the batches which still exist, the guesses we have are still associated with the same batches. 
            //Assuming that there are no massive changes in the constraint set, this is a pretty good guess.
            var batchCount = batchBoundaries.Count;
            var oldBatchCount = workerContexts[0].BlocksOwnedInBatches.Count;
            if (batchCount > oldBatchCount)
            {
                //There are more batches now than before. Need to initialize them on every existing worker.
                for (int contextIndex = 0; contextIndex < Math.Min(workerCount, workerContexts.Count); ++contextIndex)
                {
                    ref var context = ref workerContexts[contextIndex];
                    //The new frame has more constraint batches. We need to set up new batch ownership lists for each worker.
                    //Note that there's no need to ensure the capacity on each batch block ownership list that already exists; the maxmimum required capacity is constant and was already set when the list was created.
                    context.BlocksOwnedInBatches.EnsureCapacity(batchCount, bufferPool.SpecializeFor<QuickList<int, Buffer<int>>>());
                    context.BlocksOwnedInBatches.Count = batchCount;
                    for (int i = oldBatchCount; i < batchCount; ++i)
                    {
                        //Note that we allocate enough space to hold every block in the worst case. That could potentially happen due to workstealing.
                        QuickList<int, Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), maximumBlocksPerBatch, out context.BlocksOwnedInBatches[i]);
                    }
                }
            }
            else if (batchCount < oldBatchCount)
            {
                //Less batches now exist. Get rid of all the extras.
                for (int contextIndex = 0; contextIndex < Math.Min(workerCount, workerContexts.Count); ++contextIndex)
                {
                    ref var context = ref workerContexts[contextIndex];
                    for (int i = batchCount; i < oldBatchCount; ++i)
                    {
                        context.BlocksOwnedInBatches[i].Dispose(bufferPool.SpecializeFor<int>());
                    }
                    context.BlocksOwnedInBatches.Count = batchCount;
                    //We may be able to shrink the size.
                    context.BlocksOwnedInBatches.Compact(bufferPool.SpecializeFor<QuickList<int, Buffer<int>>>());
                }
            }

            //In the event that the worker count changes, the easiest thing to do is just throw out all previous guesses.
            //This isn't optimal, but worker count changes should be really, really rare.
            //Note that this doubles as initialization.
            if (workerContexts.Count != workerCount)
            {
                if (workerCount > workerContexts.Count)
                {
                    //Add any new contexts that we need.
                    workerContexts.EnsureCapacity(workerCount, bufferPool.SpecializeFor<Worker>());
                    for (int i = workerContexts.Count; i < workerCount; ++i)
                    {
                        Worker context;
                        QuickList<QuickList<int, Buffer<int>>, Buffer<QuickList<int, Buffer<int>>>>.Create(
                            bufferPool.SpecializeFor<QuickList<int, Buffer<int>>>(), batchBoundaries.Count, out context.BlocksOwnedInBatches);
                        for (int j = 0; j < batchBoundaries.Count; ++j)
                        {
                            //Note that we allocate enough space to hold every block in the worst case. That could potentially happen due to workstealing.
                            QuickList<int, Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), maximumBlocksPerBatch, out context.BlocksOwnedInBatches[j]);
                        }
                        workerContexts.AddUnsafely(ref context);
                    }
                }
                else
                {
                    //Get rid of every context beyond those that we need.
                    for (int i = workerCount; i < workerContexts.Count; ++i)
                    {
                        ref var worker = ref workerContexts[i];
                        for (int j = 0; j < workerContexts[i].BlocksOwnedInBatches.Count; ++j)
                        {
                            worker.BlocksOwnedInBatches[j].Dispose(bufferPool.SpecializeFor<int>());
                        }
                        worker.BlocksOwnedInBatches.Dispose(bufferPool.SpecializeFor<QuickList<int, Buffer<int>>>());
                    }
                    workerContexts.Count = workerCount;
                    workerContexts.Compact(bufferPool.SpecializeFor<Worker>());
                }
                //Note that we don't bother compacting the individual worker's OwnedBlocks that persist.
                //The number of blocks allocated to each worker is bounded to a low constant value.

                //Since we don't have a decent guess of where to put blocks, just evenly distribute them and shrug. Next frame will give us more information.
                int blockIndex = 0;
                for (int batchIndex = 0; batchIndex < batchBoundaries.Count; ++batchIndex)
                {
                    //Spread the blocks of the batch across the set of workers.
                    var start = batchIndex >= 0 ? batchBoundaries[batchIndex - 1] : 0;
                    var count = batchBoundaries[batchIndex] - start;

                    var targetBlocksPerWorker = workBlocks.Count / workerCount;
                    var remainder = workBlocks.Count - workerCount * targetBlocksPerWorker;
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                    {
                        ref var ownedBlocksInBatch = ref workerContexts[workerIndex].BlocksOwnedInBatches[batchIndex];
                        var blockCount = remainder-- > 0 ? targetBlocksPerBatchPerWorker + 1 : targetBlocksPerBatchPerWorker;
                        var workerRegionEnd = blockIndex + blockCount;
                        for (; blockIndex < workerRegionEnd; ++blockIndex)
                        {
                            ownedBlocksInBatch.AddUnsafely(blockIndex);
                        }
                    }
                }
                return;
            }
            //We still have the same number of threads, so we should be able to get some inspiration from the previous frame's load balancing result.
            //Note that the constraint set may have changed arbitrarily since the previous frame, so this is not a guarantee of a good load balance.
            //It just relies on temporal coherence.

            //The idea here is to allocate blocks to workers in proportion to how many blocks they were able to handle in the previous solve.
            //This is done per-batch; each batch could have significantly different per block costs due to different constraints being involved.
            {
                var blockIndex = 0;
                for (int batchIndex = 0; batchIndex < batchCount; ++batchIndex)
                {
                    var totalBlockCount = 0;
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                    {
                        totalBlockCount += workerContexts[workerIndex].BlocksOwnedInBatches[batchIndex].Count;
                    }
                    //Use the fraction of the previous frame's block owned by each worker to determine how many blocks it should now have.
                    //Two phases: the initial estimate, followed by the distribution of the remainder.
                    var allocatedBlockCount = 0;
                    var blockStartForBatch = batchIndex >= 0 ? batchBoundaries[batchIndex - 1] : 0;
                    var blockCountForBatch = batchBoundaries[batchIndex] - blockStartForBatch;
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                    {
                        ref var ownedBlocksInBatch = ref workerContexts[workerIndex].BlocksOwnedInBatches[batchIndex];
                        var blockCountForWorker = (blockCountForBatch * ownedBlocksInBatch.Count) / totalBlockCount;
                        allocatedBlockCount += ownedBlocksInBatch.Count = blockCountForWorker;
                    }
                    var targetWorker = 0;
                    while (allocatedBlockCount < blockCountForBatch)
                    {
                        ++workerContexts[targetWorker++].BlocksOwnedInBatches[batchIndex].Count;
                        ++allocatedBlockCount;
                    }

                    //We have reasonably distributed counts. Now, stick contiguous blobs of indices into each worker's list to fill it up. 
                    for (int workerIndex = 0; workerIndex < workerCount; ++workerIndex)
                    {
                        ref var ownedBlocksInBatch = ref workerContexts[workerIndex].BlocksOwnedInBatches[batchIndex];
                        for (int i = 0; i < ownedBlocksInBatch.Count; ++i)
                            ownedBlocksInBatch[i] = blockIndex++;
                    }
                    Debug.Assert(blockIndex == batchBoundaries[batchIndex]);
                }
            }
            //Note that we have no guarantee that every thread will actually have a block to work on. That's okay- everything still works. The empty threads will be doing
            //a lot of work spinning their wheels, but at those scales, the total simulation time will be measured in microseconds anyway. It's not a big concern.
            //The goal for multithreading is for enormous simulations to be reasonably fast, not for extremely tiny simulations to update at absurd rates.
        }


        /// <summary>
        /// Returns the temporary resources to the pool.
        /// </summary>
        static void Return(BufferPool bufferPool,
            ref QuickList<WorkBlock, Buffer<WorkBlock>> workBlocks,
            ref QuickList<int, Buffer<int>> blockClaims,
            ref QuickList<int, Buffer<int>> batchBoundaries)
        {
            workBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            blockClaims.Dispose(bufferPool.SpecializeFor<int>());
            batchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
        }


        //Multithreading delegate parameters.
        struct MultithreadingParameters
        {
            public float Dt;
            public float InverseDt;
            public QuickList<Worker, Buffer<Worker>> Workers;
            public QuickList<WorkBlock, Buffer<WorkBlock>> WorkBlocks;
            public QuickList<int, Buffer<int>> BlockClaims;
            public QuickList<int, Buffer<int>> BatchBoundaries;
            public QuickList<int, Buffer<int>> StageRemainingBlocks;
            public int WorkerCompletedCount;
            public int WorkerCount;
        }
        MultithreadingParameters context;

        interface IBlockFunction
        {
            //Note that the second parameter can either be an index into the ownedBlocks, or into the workBlocks.
            //It depends on the user. TryWorkSteal provides an index into the workBlocks, while an owned execution uses 
            void ExecuteBlock(TypeBatch batch, int startBundle, int endBundle);
        }

        struct PrestepFunction : IBlockFunction
        {
            public float Dt;
            public float InverseDt;
            public BodyInertias[] Inertias;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ExecuteBlock(TypeBatch batch, int startBundle, int endBundle)
            {
                batch.Prestep(Inertias, Dt, InverseDt, startBundle, endBundle);
            }
        }

        struct WarmStartFunction : IBlockFunction
        {
            public BodyVelocities[] Velocities;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ExecuteBlock(TypeBatch batch, int startBundle, int endBundle)
            {
                batch.WarmStart(Velocities, startBundle, endBundle);
            }
        }

        struct SolveFunction : IBlockFunction
        {
            public BodyVelocities[] Velocities;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void ExecuteBlock(TypeBatch batch, int startBundle, int endBundle)
            {
                batch.SolveIteration(Velocities, startBundle, endBundle);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool TryExecuteBlock<TBlockFunction>(ref TBlockFunction function, int slot, int claimedState, int clearedState, ref int remainingBlocks)
            where TBlockFunction : IBlockFunction
        {
            if (Interlocked.CompareExchange(ref context.BlockClaims[slot], claimedState, clearedState) == clearedState)
            {
                //This worker now owns the slot; we can freely execute it.
                ref var block = ref context.WorkBlocks[slot];
                function.ExecuteBlock(Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex], block.StartBundle, block.End);
                //Decrement the available block count to allow workers to stop trying to steal. 
                //Note that the fact that this decrement is not atomic with the CAS is not a correctness issue-
                //it may just cause some threads to do an extra claims test or two pointlessly. The syncs required to avoid that cost more.
                //(TODO: You COULD technically flip the conditions- decrement first, claim second. But there's no guarantee that we will be able to claim after the decrement,
                //so that decremented slot might end up being a steal later on. A little more complex, but quite possibly worth it overall.)
                Interlocked.Decrement(ref remainingBlocks);
                return true;
            }
            return false;
        }

        interface IOnStealFunction
        {
            void OnSteal(ref QuickList<int, Buffer<int>> ownedBlocks, int stolenSlot);
        }

        struct DoNothingOnSteal : IOnStealFunction
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnSteal(ref QuickList<int, Buffer<int>> ownedBlocks, int stolenSlot)
            {
            }
        }
        struct TakeOwnershipOnSteal : IOnStealFunction
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnSteal(ref QuickList<int, Buffer<int>> ownedBlocks, int stolenSlot)
            {
                //We built the ownership lists to be able to hold all blocks simultaneously, so this is safe.
                ownedBlocks.AddUnsafely(stolenSlot);
            }
        }


        void WorkStealAndSync<TBlockFunction, TOnStealSuccessFunction>(ref TBlockFunction blockFunction, ref QuickList<int, Buffer<int>> ownedBlocks,
           int blocksStart, int blocksEnd,
           ref int syncStageIndex, ref int claimedState, ref int clearedState, ref int remainingBlocks)
           where TBlockFunction : IBlockFunction
           where TOnStealSuccessFunction : IOnStealFunction
        {
            //We're out of our allocated blocks. Could we steal any?
            if (Volatile.Read(ref remainingBlocks) > 0)
            {
                //Start trying to steal at the index just before the owned batch blob. Scan backwards.
                //Starting at the end and working backwards increases the probability that an unclaimed block can be found.
                int stealSlot = ownedBlocks.Count > 0 ? ownedBlocks[0] - 1 : -1;
                if (stealSlot < blocksStart)
                    stealSlot = blocksEnd;
                while (Volatile.Read(ref remainingBlocks) > 0)
                {
                    stealSlot = stealSlot == blocksStart ? blocksEnd : stealSlot - 1;
                    if (TryExecuteBlock(ref blockFunction, stealSlot, claimedState, clearedState, ref remainingBlocks))
                    {
                        default(TOnStealSuccessFunction).OnSteal(ref ownedBlocks, stealSlot);
                    }
                }
            }

            //Swap the ownership flags.
            var temp = clearedState;
            clearedState = claimedState;
            claimedState = temp;

            //No more work is available to claim, but not every thread is necessarily done with the work they claimed. So we need a dedicated sync- upon completing its local work,
            //a worker increments the 'workerCompleted' counter, and the spins on that counter reaching workerCount * stageIndex.
            ++syncStageIndex;
            var neededCompletionCount = context.WorkerCount * syncStageIndex;
            if (Interlocked.Increment(ref context.WorkerCompletedCount) != neededCompletionCount)
            {
                var wait = new SpinWait();
                while (context.WorkerCompletedCount != neededCompletionCount)
                    wait.SpinOnce();
            }
        }
        

        void Work(int workerIndex)
        {
            ref var worker = ref context.Workers[workerIndex];
            Debug.Assert(worker.BlocksOwnedInBatches.Count > 0, "Since have to check the batch count at some point, you might as well check it before dispatch to avoid wasting time.");

            //Prestep first.
            //The prestep does not need to sync at the border of batch boundaries. Just do every work block.
            //The CAS will try to swap from cleared to owned, but after each loop we have to flip the cleared and owned states.
            //This avoids the need for an explicit clear.
            var claimedState = 1;
            var clearedState = 0;
            int syncStageIndex = 0;
            {
                ref var remainingBlocks = ref context.StageRemainingBlocks[syncStageIndex];

                //TODO: you may actually want to try a 'dumb' claim system for presteps; you could use a single interlocked increment to claim slots.
                //This would mean a worker would do a prestep of a block that it probably won't be responsible for during the iteration phase, but 
                //the simpler sync might be a net win. Unclear without testing.
                var prestepFunction = new PrestepFunction { Dt = context.Dt, InverseDt = context.InverseDt, Inertias = bodies.LocalInertiaBundles };
                for (int batchIndex = 0; batchIndex < worker.BlocksOwnedInBatches.Count && Volatile.Read(ref remainingBlocks) > 0; ++batchIndex)
                {
                    ref var batchOwnedBlockList = ref worker.BlocksOwnedInBatches[batchIndex];
                    for (int ownedIndex = 0; ownedIndex < batchOwnedBlockList.Count && Volatile.Read(ref remainingBlocks) > 0; ++ownedIndex)
                    {
                        TryExecuteBlock(ref prestepFunction, batchOwnedBlockList[ownedIndex], claimedState, clearedState, ref remainingBlocks);
                    }
                }
                //By this point, the worker has executed (or attempted to execute) every one of their allocated work blocks. Due to differences in the difficulty of different blocks,
                //some workers may still not be done. This thread should try to help out with workstealing.

                WorkStealAndSync<PrestepFunction, DoNothingOnSteal>(ref prestepFunction, ref worker.BlocksOwnedInBatches[worker.BlocksOwnedInBatches.Count - 1],
                    0, context.WorkBlocks.Count - 1, ref syncStageIndex, ref claimedState, ref clearedState, ref remainingBlocks);

            }
            //A sync is also required for the prestep->warmstartbatch0 transition if any of the worker's prestep blocks were stolen. 
            //While we could check that condition easily, we would also have to stop the local worker from workstealing any other blocks unless that block's prestep has completed.
            //That would require either checking per-block prestep completion or all-prestep completion upon every warmstart worksteal.
            //It might be worth looking into later, but for now, let's just sync always.

            //TODO: If the time step duration has changed since the previous frame, the cached accumulated impulses should be scaled prior to application.
            //This isn't critical, but it does help stability.

            //Warm start second.
            var warmStartFunction = new WarmStartFunction { Velocities = bodies.VelocityBundles };
            for (int batchIndex = 0; batchIndex < worker.BlocksOwnedInBatches.Count; ++batchIndex)
            {
                ref var remainingBlocks = ref context.StageRemainingBlocks[syncStageIndex];
                ref var ownedBlocks = ref worker.BlocksOwnedInBatches[batchIndex];
                for (int ownedIndex = 0; ownedIndex < ownedBlocks.Count && Volatile.Read(ref remainingBlocks) > 0; ++ownedIndex)
                {
                    TryExecuteBlock(ref warmStartFunction, ownedBlocks[ownedIndex], claimedState, clearedState, ref remainingBlocks);
                }

                WorkStealAndSync<WarmStartFunction, DoNothingOnSteal>(ref warmStartFunction, ref ownedBlocks, batchIndex >= 0 ? context.BatchBoundaries[batchIndex - 1] : 0, context.BatchBoundaries[batchIndex],
                     ref syncStageIndex, ref claimedState, ref clearedState, ref remainingBlocks);
            }

            //Solve iterations last.
            var solveFunction = new SolveFunction { Velocities = bodies.VelocityBundles };
            for (int i = 0; i < iterationCount; ++i)
            {
                for (int batchIndex = 0; batchIndex < worker.BlocksOwnedInBatches.Count; ++batchIndex)
                {
                    ref var remainingBlocks = ref context.StageRemainingBlocks[syncStageIndex];
                    ref var ownedBlocks = ref worker.BlocksOwnedInBatches[batchIndex];
                    for (int ownedIndex = 0; ownedIndex < ownedBlocks.Count; ++ownedIndex)
                    {
                        if (!TryExecuteBlock(ref solveFunction, ownedBlocks[ownedIndex], claimedState, clearedState, ref remainingBlocks))
                        {
                            //The block was already claimed by another thread. Remove it from the local worker's ownership. 
                            ownedBlocks.RemoveAt(ownedIndex);
                            //Because of this, this loop CANNOT be terminated early by remainingBlocks == 0; that could result in the ownership list still containing a block that was stolen.
                            //This is the bullet we bite by not allowing other threads to modify the local worker's owned set.
                        }
                    }

                    WorkStealAndSync<SolveFunction, TakeOwnershipOnSteal>(ref solveFunction, ref ownedBlocks, batchIndex >= 0 ? context.BatchBoundaries[batchIndex - 1] : 0, context.BatchBoundaries[batchIndex],
                         ref syncStageIndex, ref claimedState, ref clearedState, ref remainingBlocks);
                }
            }
        }


    }
}
