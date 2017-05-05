using System.Runtime.CompilerServices;
using static SolverPrototype.ConstraintConnectivityGraph;
using System;
using System.Diagnostics;
using BEPUutilities2.Memory;
using BEPUutilities2.Collections;
using System.Runtime.InteropServices;
using System.Threading;

namespace SolverPrototype
{
    /// <summary>
    /// Incrementally changes the layout of a set of bodies to minimize the cache misses associated with the solver and other systems that rely on connection following.
    /// </summary>
    public class BodyLayoutOptimizer
    {
        Bodies bodies;
        ConstraintConnectivityGraph graph;
        Solver solver;
        public BodyLayoutOptimizer(Bodies bodies, ConstraintConnectivityGraph graph, Solver solver, BufferPool pool)
        {
            this.bodies = bodies;
            this.graph = graph;
            this.solver = solver;

            islandEnumerator = new PartialIslandDFSEnumerator
            {
                bodies = bodies,
                graph = graph,
                solver = solver,
                pool = pool.SpecializeFor<int>()
            };
            QuickList<int, Buffer<int>>.Create(islandEnumerator.pool, 32, out islandEnumerator.traversalStack);
        }

        //TODO: Note that there are a few ways we can do multithreading. The naive way is to just apply locks on all the nodes affected by an optimization candidate.
        //That'll work reasonably well, though a major fraction of the total execution time will be tied up in those locks. Might not scale particularly well 
        //in the context of a big multiprocessor server.
        //At the cost of convergence speed, you can instead choose to optimize region by region. 
        //An optimizing thread can be given a subset of all bodies with guaranteed exclusive access by scheduling.
        //While that thread can't necessarily swap bodies to where the locking version would, it can make progress toward the goal over time.
        //Rather than 'pulling', it would 'push'- find a parent to the left, and go as far to the left towards it as possible.
        //You'd have to be a bit tricky to ensure that all bodies will move towards it (rather than just one that continually gets swapped around), and the end behavior
        //could be a little different, but it might end up being faster overall due to the lack of contention.
        //The same concept could apply to the broad phase optimizer too, though it's a little easier there (and the naive locking requirements are more complex per swap, too).

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdateConstraintsForBodyMemoryMove(int bodyIndex, int newBodyIndex, ConstraintConnectivityGraph graph, Solver solver)
        {
            ref var list = ref graph.GetConstraintList(bodyIndex);
            for (int i = 0; i < list.Count; ++i)
            {
                ref var constraint = ref list[i];
                solver.UpdateForBodyMemoryMove(constraint.ConnectingConstraintHandle, constraint.BodyIndexInConstraint, newBodyIndex);
            }

        }

        public static void SwapBodyLocation(Bodies bodies, ConstraintConnectivityGraph graph, Solver solver, int a, int b)
        {
            Debug.Assert(a != b, "Swapping a body with itself isn't meaningful. Whaddeyer doin?");
            //Enumerate the bodies' current set of constraints, changing the reference in each to the new location.
            //Note that references to both bodies must be changed- both bodies moved!
            //This function does not update the actual position of the list in the graph, so we can modify both without worrying about invalidating indices.
            UpdateConstraintsForBodyMemoryMove(a, b, graph, solver);
            UpdateConstraintsForBodyMemoryMove(b, a, graph, solver);

            //Update the body and graph locations.
            bodies.Swap(a, b);
            graph.SwapBodies(a, b);
        }

        int dumbBodyIndex = 0;

        struct DumbIncrementalEnumerator : IForEach<int>
        {
            public Bodies bodies;
            public ConstraintConnectivityGraph graph;
            public Solver solver;
            public int slotIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connectedBodyIndex > slotIndex)
                {
                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.

                    //TODO: If we end up choosing to go with the dumb optimizer, you can almost certainly improve this implementation-
                    //this version goes through all the effort of diving into the type batches for references, then does it all again to move stuff around.
                    //A hardcoded swapping operation could do both at once, saving a few indirections.
                    //It won't be THAT much faster- every single indirection is already cached- but it's probably still worth it.
                    //(It's not like this implementation with all of its nested enumerators is particularly simple or clean.)
                    var newLocation = slotIndex++;
                    //Note that graph.EnumerateConnectedBodies explicitly excludes the body whose constraints we are enumerating, 
                    //so we don't have to worry about having the rug pulled by this list swap.
                    //(Also, !(x > x) for many values of x.)
                    SwapBodyLocation(bodies, graph, solver, connectedBodyIndex, newLocation);

                    //slotIndex = int.MaxValue;
                }
            }
        }
        public void DumbIncrementalOptimize()
        {
            //All this does is look for any bodies which are to the right of a given body. If it finds one, it pulls it to be adjacent.
            //This converges at the island level- that is, running this on a static topology of simulation islands will eventually result in 
            //the islands being contiguous in memory, and at least some connected bodies being adjacent to each other.
            //However, within the islands, it may continue to unnecessarily swap objects around as bodies 'fight' for ownership.
            //One body doesn't know that another body has already claimed a body as a child, so this can't produce a coherent unique traversal order.
            //(In fact, it won't generally converge even with a single one dimensional chain of bodies.)

            //This optimization routine requires much less overhead than other options, like full island traversals. We only request the connections of a single body,
            //and the swap count is limited to the number of connected bodies.

            //Note that this first implementation really does not care about performance. Just looking for the performance impact on the solver at this point.

            if (dumbBodyIndex >= bodies.BodyCount - 1)
                dumbBodyIndex = 0;

            var enumerator = new DumbIncrementalEnumerator();
            enumerator.bodies = bodies;
            enumerator.graph = graph;
            enumerator.solver = solver;
            enumerator.slotIndex = dumbBodyIndex + 1;
            graph.EnumerateConnectedBodies(dumbBodyIndex, ref enumerator);

            ++dumbBodyIndex;

        }

        int sortingBodyIndex = 0;

        struct CollectingEnumerator : IForEach<int>
        {
            public int minimumIndex;
            public QuickList<int, Buffer<int>> bodyIndices;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connectedBodyIndex > minimumIndex)
                {
                    bodyIndices.AddUnsafely(connectedBodyIndex);
                }
            }
        }
        public void SortingIncrementalOptimize(BufferPool rawPool)
        {
            //All this does is look for any bodies which are to the right of a given body. If it finds one, it pulls it to be adjacent.
            //This converges at the island level- that is, running this on a static topology of simulation islands will eventually result in 
            //the islands being contiguous in memory, and at least some connected bodies being adjacent to each other.
            //However, within the islands, it may continue to unnecessarily swap objects around as bodies 'fight' for ownership.
            //One body doesn't know that another body has already claimed a body as a child, so this can't produce a coherent unique traversal order.
            //(In fact, it won't generally converge even with a single one dimensional chain of bodies.)

            //This optimization routine requires much less overhead than other options, like full island traversals. We only request the connections of a single body,
            //and the swap count is limited to the number of connected bodies.

            //Note that this first implementation really does not care about performance. Just looking for the performance impact on the solver at this point.

            if (sortingBodyIndex >= bodies.BodyCount - 1)
                sortingBodyIndex = 0;

            var enumerator = new CollectingEnumerator();
            enumerator.minimumIndex = sortingBodyIndex;
            var pool = rawPool.SpecializeFor<int>();
            QuickList<int, Buffer<int>>.Create(pool, graph.GetConstraintList(sortingBodyIndex).Count, out enumerator.bodyIndices);

            graph.EnumerateConnectedBodies(sortingBodyIndex, ref enumerator);
            if (enumerator.bodyIndices.Count > 0)
            {
                pool.Take(enumerator.bodyIndices.Count, out var sourceIndices);
                var comparer = new PrimitiveComparer<int>();
                InsertionSort.Sort(ref enumerator.bodyIndices[0], ref sourceIndices[0], 0, enumerator.bodyIndices.Count - 1, ref comparer);
                int targetIndex = sortingBodyIndex;
                for (int i = 0; i < enumerator.bodyIndices.Count; ++i)
                {
                    ++targetIndex;
                    //Because the list is sorted, it's not possible for the swap target location to be greater than the swap source location.
                    Debug.Assert(targetIndex <= enumerator.bodyIndices[i]);
                    SwapBodyLocation(bodies, graph, solver, enumerator.bodyIndices[i], ++targetIndex);
                }
                pool.Return(ref sourceIndices);
            }
            enumerator.bodyIndices.Dispose(pool);
            ++sortingBodyIndex;
        }

        struct PartialIslandDFSEnumerator : IForEach<int>
        {
            public Bodies bodies;
            public ConstraintConnectivityGraph graph;
            public Solver solver;
            //We effectively do a full traversal over multiple frames. We have to store the stack for this to work.
            public QuickList<int, Buffer<int>> traversalStack;
            public BufferPool<int> pool;
            //The target index is just the last recorded exclusive endpoint of the island. In other words, it's the place where the next island body will be put.
            public int targetIndex;
            public int currentBodyIndex;
            public int maximumBodiesToVisit;
            public int visitedBodyCount;

            public void LoopBody(int connectedBodyIndex)
            {
                //Should this node be swapped into position? 
                if (connectedBodyIndex > targetIndex)
                {
                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.
                    var newLocation = targetIndex++;
                    Debug.Assert(newLocation > currentBodyIndex, "The target index should always progress ahead of the traversal. Did something get reset incorrectly?");
                    SwapBodyLocation(bodies, graph, solver, connectedBodyIndex, newLocation);
                    //Note that we mark the new location for traversal, since it was moved.
                    traversalStack.Add(newLocation, pool);
                }
                else if (connectedBodyIndex > currentBodyIndex)
                {
                    //While this body should not be swapped because it has already been swapped by an earlier traversal visit, 
                    //it is still a candidate for continued traversal. It might have children that cannot be reached by other paths.
                    traversalStack.Add(connectedBodyIndex, pool);
                }
            }
        }
        PartialIslandDFSEnumerator islandEnumerator;

        public void PartialIslandOptimizeDFS(int maximumBodiesToVisit = 32)
        {
            //With the observation that the full island DFS traversal is a decent cache optimization heuristic, attempt to do the same thing except spread over multiple frames.
            //This can clearly get invalidated by changes to the topology between frames, but temporary suboptimality will not cause correctness problems.

            //A few important implementation notes:
            //1) The target index advances with every swap performed.
            //2) "Visited" bodies are not explicitly tracked. Instead, the traversal will refuse to visit any bodies to the left of the current body.
            //Further, no swaps will occur with any body to the left of the target index.
            //Any body before the target index has already been swapped into position by an earlier traversal (heuristically speaking).
            //3) Swaps are performed inline, eliminating the need for any temporary body storage (or long-term locks in the multithreaded implementation).
            islandEnumerator.visitedBodyCount = 0;
            islandEnumerator.maximumBodiesToVisit = maximumBodiesToVisit;
            //First, attempt to continue any previous traversals.
            do
            {
                if (islandEnumerator.targetIndex >= bodies.BodyCount)
                {
                    //The target index has walked outside the bounds of the body set. While we could wrap around and continue, that would a different heuristic
                    //for swapping and visitation- currently we use 'to the right' which isn't well defined on a ring.
                    //Instead, for simplicity's sake, the target simply resets to the first index, and the traversal stack gets cleared.
                    islandEnumerator.targetIndex = 0;
                    islandEnumerator.traversalStack.Count = 0;
                }
                if (islandEnumerator.traversalStack.Count == 0)
                {
                    //There is no active traversal. Start one.
                    islandEnumerator.traversalStack.Add(islandEnumerator.targetIndex++, islandEnumerator.pool);
                }
                islandEnumerator.traversalStack.Pop(out islandEnumerator.currentBodyIndex);
                //It's possible for the target index to fall behind if no swaps are needed. 
                islandEnumerator.targetIndex = Math.Max(islandEnumerator.targetIndex, islandEnumerator.currentBodyIndex + 1);
                graph.EnumerateConnectedBodies(islandEnumerator.currentBodyIndex, ref islandEnumerator);
                ++islandEnumerator.visitedBodyCount;
            } while (islandEnumerator.visitedBodyCount < islandEnumerator.maximumBodiesToVisit);
        }


        //Note that external systems which have to respond to body movements will generally be okay without their own synchronization in multithreaded cache optimization.
        //For example, the constraint handle, type batch index, index in type batch, and body index in constraint do not change. If we properly synchronize
        //the body accesses, then the changes to constraint bodies will be synchronized too.

        //Of course, it may turn out that the overhead of the synchronization (a per body interlocked call) is so high relative to the actual 'work' being done that
        //a single threaded implementation running at the same time as some other compatible stage just vastly outperforms it. It is very likely that the multithreaded version
        //will be 2-6 times slower than the single threaded version per thread.



        struct ClaimConnectedBodiesEnumerator : IForEach<int>
        {
            public Bodies Bodies;
            public ConstraintConnectivityGraph Graph;
            public Solver Solver;
            /// <summary>
            /// The claim states for every body in the simulation.
            /// </summary>
            public Buffer<int> ClaimStates;
            /// <summary>
            /// The set of claims owned by the current worker.
            /// </summary>
            public QuickList<int, Buffer<int>> WorkerClaims;
            public int ClaimIdentity;
            public bool AllClaimsSucceeded;

            public bool TryClaim(int index)
            {
                var preclaimValue = Interlocked.CompareExchange(ref ClaimStates[index], ClaimIdentity, 0);
                if (preclaimValue == 0)
                {
                    Debug.Assert(WorkerClaims.Count < WorkerClaims.Span.Length,
                        "The claim enumerator should never be invoked if the worker claims buffer is too small to hold all the bodies.");
                    WorkerClaims.AddUnsafely(index);
                }
                else if (preclaimValue != ClaimIdentity)
                {
                    //Note that it only fails when it's both nonzero AND not equal to the claim identity. It means it's claimed by a different worker.
                    return false;
                }
                return true;
            }
            /// <summary>
            /// Because new claims are appended, and a failed claim always results in the removal of a contiguous set of trailing indices, it acts like a stack.
            /// This pops off a number of the most recent claim indices.
            /// </summary>
            /// <param name="workerClaimsToPop">Number of claims to remove from the end of the worker claims list.</param>
            public void Unclaim(int workerClaimsToPop)
            {
                Debug.Assert(workerClaimsToPop <= WorkerClaims.Count, "The pop request should never exceed the accumulated claims.");
                for (int i = 0; i < workerClaimsToPop; ++i)
                {
                    WorkerClaims.Pop(out var poppedIndex);
                    //We don't need to do anything special here, provided a reasonably strong memory model. Just release the slot.
                    ClaimStates[poppedIndex] = 0;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //TODO: If you end up going with this approach, you should probably use a IBreakableForEach instead to avoid unnecessary traversals.
                if (AllClaimsSucceeded)
                {
                    if (!TryClaim(connectedBodyIndex))
                        AllClaimsSucceeded = false;
                }
            }

        }

        struct MultithreadedDumbIncrementalEnumerator : IForEach<int>
        {
            public ClaimConnectedBodiesEnumerator ClaimEnumerator;
            public int slotIndex;
            public int TotalNeededClaimCount;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connectedBodyIndex > slotIndex)
                {
                    var newLocation = slotIndex++;
                    //We must claim both the swap source, target, AND all of the bodies connected to them.
                    //(If we didn't claim the conected bodies, the changes to the constraint body indices could break the traversal.)
                    //(We don't do a constraint claims array directly because there is no single contiguous and easily indexable set of constraints.)
                    var previousClaimCount = ClaimEnumerator.WorkerClaims.Count;
                    var neededSize = previousClaimCount + 2 +
                        ClaimEnumerator.Graph.GetConstraintList(connectedBodyIndex).Count +
                        ClaimEnumerator.Graph.GetConstraintList(connectedBodyIndex).Count;
                    //We accumulate the number of claims needed so that later updates can expand the claim array if necessary.
                    //(This should be exceptionally rare so long as a decent initial size is chosen- unless you happen to attach 5000 constraints
                    //to a single object. Which is a really bad idea.)
                    TotalNeededClaimCount += neededSize;
                    if (neededSize > ClaimEnumerator.WorkerClaims.Span.Length)
                    {
                        //This body can't be claimed; we don't have enough space left.
                        return;
                    }
                    if (!ClaimEnumerator.TryClaim(connectedBodyIndex))
                        return;
                    if (!ClaimEnumerator.TryClaim(newLocation))
                    {
                        //If this claim failed but the previous succeeded, we should unclaim the remote location.
                        ClaimEnumerator.Unclaim(1);
                        return;
                    }
                    ClaimEnumerator.Graph.EnumerateConnectedBodies(connectedBodyIndex, ref ClaimEnumerator);
                    if (!ClaimEnumerator.AllClaimsSucceeded)
                    {
                        ClaimEnumerator.Unclaim(ClaimEnumerator.WorkerClaims.Count - previousClaimCount);
                        return;
                    }
                    ClaimEnumerator.Graph.EnumerateConnectedBodies(newLocation, ref ClaimEnumerator);
                    if (!ClaimEnumerator.AllClaimsSucceeded)
                    {
                        ClaimEnumerator.Unclaim(ClaimEnumerator.WorkerClaims.Count - previousClaimCount);
                        return;
                    }

                    //At this point, we have claimed both swap targets and all their satellites. Can actually do the swap.

                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.
                    SwapBodyLocation(ClaimEnumerator.Bodies, ClaimEnumerator.Graph, ClaimEnumerator.Solver, connectedBodyIndex, newLocation);

                }
            }
        }
        //Avoid a little pointless false sharing with padding.
        [StructLayout(LayoutKind.Explicit, Size = 128)]
        struct Worker
        {
            [FieldOffset(0)]
            public int Index;
            [FieldOffset(4)]
            public int LargestNeededClaimCount;
            [FieldOffset(8)]
            public int CompletedJobs;
            [FieldOffset(16)]
            public QuickList<int, Buffer<int>> WorkerClaims;
        }
        int remainingOptimizationAttemptCount;
        Buffer<Worker> workers;

        void MultithreadedDumbIncrementalOptimize(int workerIndex)
        {
            var enumerator = new MultithreadedDumbIncrementalEnumerator();
            ref var worker = ref workers[workerIndex];
            enumerator.ClaimEnumerator = new ClaimConnectedBodiesEnumerator
            {
                Bodies = bodies,
                Graph = graph,
                Solver = solver,
                ClaimStates = claims,
                WorkerClaims = worker.WorkerClaims,
                ClaimIdentity = workerIndex + 1,
            };
            //Note that these are optimization *attempts*. If we fail due to contention, that's totally fine. We'll get around to it later.
            //Remember, this is strictly a performance oriented heuristic. Even if all bodies are completely scrambled, it will still produce perfectly correct results.
            while (Interlocked.Decrement(ref remainingOptimizationAttemptCount) >= 0)
            {
                enumerator.TotalNeededClaimCount = 0;
                Debug.Assert(worker.Index < bodies.BodyCount - 2, "The scheduler shouldn't produce optimizations targeting the last two slots- no swaps are possible.");
                var optimizationTarget = worker.Index++;
                //There's no reason to target the last two slots. No swaps are possible.
                if (worker.Index >= bodies.BodyCount - 2)
                    worker.Index = 0;
                enumerator.slotIndex = optimizationTarget + 1;
                //We don't want to let other threads yank the claim origin away from us while we're enumerating over its connections. That would be complicated to deal with.
                if (!enumerator.ClaimEnumerator.TryClaim(optimizationTarget))
                    continue; //Couldn't grab the origin; just move on.
                graph.EnumerateConnectedBodies(optimizationTarget, ref enumerator);

                //The optimization for this object is complete. We should relinquish all existing claims.
                Debug.Assert(enumerator.ClaimEnumerator.WorkerClaims.Count == 1 && enumerator.ClaimEnumerator.WorkerClaims[0] == optimizationTarget,
                    "The only remaining claim should be the optimization target; all others should have been relinquished within the inner enumerator.");
                enumerator.ClaimEnumerator.Unclaim(1);

                if (enumerator.TotalNeededClaimCount > worker.LargestNeededClaimCount)
                    worker.LargestNeededClaimCount = enumerator.TotalNeededClaimCount;
            }

        }

        //This claims set is a part of the optimizer for now, but if there is ever a time where you might want a per body claims set for some other multithreaded purpose,
        //move it into the Bodies class instead. It tends to resize with the body count, so it makes sense to bundle it if there is any sharing at all.
        //Note that claims are peristent because clearing a few kilobytes every frame isn't free. (It's not expensive, either, but it's not free.)
        Buffer<int> claims;
        //We pick an extremely generous value to begin with because there's not much reason not to. This avoids problems in most reasonable simulations.
        int workerClaimsBufferSize = 1; //TODO: Just set it to 512 or something once it's confirmed to be working.
        public void DumbOptimizeMultithreaded(int optimizationCount, IThreadPool threadPool, BufferPool rawPool)
        {
            if (SpanHelper.GetContainingPowerOf2(bodies.BodyCount) > claims.Length || claims.Length > bodies.BodyCount * 2)
            {
                //We need a new claims buffer. Get rid of the old one.
                Dispose(rawPool);
                rawPool.SpecializeFor<int>().Take(bodies.BodyCount, out claims);
                //Claims need to be zeroed so that workers can claim by identity.
                //Go ahead and clear the full buffer so we don't have to worry about later body additions resulting in accesses to uncleared memory.
                //Easier to clear upfront than track every single add.
                claims.Clear(0, claims.Length);
            }
            rawPool.SpecializeFor<Worker>().Take(threadPool.ThreadCount, out workers);

            //Note that we ignore the last two slots as optimization targets- no swaps are possible.
            if (dumbBodyIndex >= bodies.BodyCount - 2)
                dumbBodyIndex = 0;
            //Each worker is assigned a start location evenly spaced from other workers. The less interference, the better.
            var spacingBetweenWorkers = bodies.BodyCount / threadPool.ThreadCount;
            var spacingRemainder = spacingBetweenWorkers - spacingBetweenWorkers * threadPool.ThreadCount;
            var optimizationsPerWorker = optimizationCount / threadPool.ThreadCount;
            var optimizationsRemainder = optimizationCount - optimizationsPerWorker * threadPool.ThreadCount;
            int nextStartIndex = dumbBodyIndex;
            for (int i = 0; i < threadPool.ThreadCount; ++i)
            {
                ref var worker = ref workers[i];
                worker.Index = nextStartIndex;
                worker.LargestNeededClaimCount = 0;
                worker.CompletedJobs = 0;
                QuickList<int, Buffer<int>>.Create(rawPool.SpecializeFor<int>(), workerClaimsBufferSize, out worker.WorkerClaims);

                nextStartIndex += spacingBetweenWorkers;
                if (--spacingRemainder >= 0)
                    ++nextStartIndex;
                //Note that we just wrap to zero rather than taking into acount the amount of spill. 
                //No real downside, and ensures that the potentially longest distance pulls get done.
                //Also note that we ignore the last two slots as optimization targets- no swaps are possible.
                if (nextStartIndex >= bodies.BodyCount - 2)
                    nextStartIndex = 0;
            }
            //Every worker moves forward from its start location by decrementing the optimization count to claim an optimization job.
            remainingOptimizationAttemptCount = Math.Min(bodies.BodyCount, optimizationCount);

            threadPool.ForLoop(0, threadPool.ThreadCount, MultithreadedDumbIncrementalOptimize);

            int lowestWorkerJobsCompleted = int.MaxValue;
            for (int i = 0; i < threadPool.ThreadCount; ++i)
            {
                ref var worker = ref workers[i];
                //Update the initial buffer size if it was too small this frame.
                if (worker.LargestNeededClaimCount > workerClaimsBufferSize)
                    workerClaimsBufferSize = worker.LargestNeededClaimCount;
                if (worker.CompletedJobs < lowestWorkerJobsCompleted)
                    lowestWorkerJobsCompleted = worker.CompletedJobs;

                Debug.Assert(worker.WorkerClaims.Count == 0, "After execution, all worker claims should be relinquished.");
                worker.WorkerClaims.Dispose(rawPool.SpecializeFor<int>());
            }

            rawPool.SpecializeFor<Worker>().Return(ref workers);

            //Push all workers forward by the amount the slowest one got done- or a fixed minimum to avoid stalls.
            //The goal is to ensure each worker gets to keep working on a contiguous blob as much as possible for higher cache coherence.
            //This isn't a hard guarantee- contested claims and thread scheduling can easily result in gaps in the optimization, but that's fine.
            //No harm to correctness; we'll eventually optimize it during a later frame.
            dumbBodyIndex += Math.Max(lowestWorkerJobsCompleted, Math.Max(1, optimizationCount / (threadPool.ThreadCount * 4)));
        }

        /// <summary>
        /// Returns the multithreaded claims array to the provided pool.
        /// </summary>
        /// <remarks>Apart from its use in the optimization function itself, this only needs to be called when the simulation is being torn down 
        /// and outstanding pinned resources need to be reclaimed (with the assumption that the underlying bufferpool will be reused).</remarks>
        /// <param name="rawPool">Pool to return the claims array to.</param>
        public void Dispose(BufferPool rawPool)
        {
            if (claims.Length > 0)
            {
                rawPool.SpecializeFor<int>().Return(ref claims);
                claims = new Buffer<int>();
            }
        }
    }
}
