using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Text;

namespace SolverPrototype
{
    internal struct SolverWorkBlock
    {
        public TypeBatch TypeBatch;
        public int StartBundle;
        public int EndBundle;
    }

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



        static void CreateWorkBlocks(int workerCount, BufferPool bufferPool,
             out QuickList<SolverWorkBlock, Buffer<SolverWorkBlock>> workBlocks, out QuickList<int, Buffer<int>> batchBoundaries)
        {
            //For now, we assume that every typebatch has roughly similar per constraint costs. This isn't actually true- some can be significantly cheaper.
        }

        static void Return(BufferPool bufferPool, ref QuickList<SolverWorkBlock, Buffer<SolverWorkBlock>> workBlocks, ref QuickList<int, Buffer<int>> batchBoundaries)
        {
            workBlocks.Dispose(bufferPool.SpecializeFor<SolverWorkBlock>());
            batchBoundaries.Dispose(bufferPool.SpecializeFor<int>());
        }
    }
}
