using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

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

        /// <summary>
        /// Creates a set of work blocks for worker threads to operate on and steal. 
        /// </summary>
        /// <param name="workerCount">Number of thread workers available.</param>
        /// <param name="solver">Solver doing the solve.</param>
        /// <param name="bufferPool">Pool to pull temporary resources from.</param>
        /// <param name="workBlocks">Set of bundle regions for threads to execute.</param>
        /// <param name="batchBoundaries">Exclusive end indices of each batch's work block region.</param>
        static void CreateWorkBlocks(int workerCount, Solver solver, BufferPool bufferPool,
             out QuickList<WorkBlock, Buffer<WorkBlock>> workBlocks, out QuickList<int, Buffer<int>> batchBoundaries)
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
        }
        /// <summary>
        /// Returns the temporary resources to the pool.
        /// </summary>
        static void Return(BufferPool bufferPool, ref QuickList<WorkBlock, Buffer<WorkBlock>> workBlocks, ref QuickList<int, Buffer<int>> batchBoundaries)
        {
            workBlocks.Dispose(bufferPool.SpecializeFor<WorkBlock>());
            batchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
        }
    }
}
