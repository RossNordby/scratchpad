﻿using System.Runtime.CompilerServices;
using static SolverPrototype.ConstraintConnectivityGraph;
using System;
using System.Diagnostics;
using BEPUutilities2.Memory;
using BEPUutilities2.Collections;

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

        int minimumBodyIndex = 0;

        struct MinimumIncrementalEnumerator : IForEach<int>
        {
            public int targetIndex;
            public int minimumIndex;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connectedBodyIndex > targetIndex && connectedBodyIndex < minimumIndex)
                {
                    minimumIndex = connectedBodyIndex;
                }
            }
        }
        public void MinimumIncrementalOptimize()
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

            if (minimumBodyIndex >= bodies.BodyCount - 1)
                minimumBodyIndex = 0;

            var enumerator = new MinimumIncrementalEnumerator();
            enumerator.targetIndex = minimumBodyIndex + 1;
            enumerator.minimumIndex = int.MaxValue;

            graph.EnumerateConnectedBodies(minimumBodyIndex, ref enumerator);

            if (enumerator.minimumIndex < int.MaxValue)
            {
                SwapBodyLocation(bodies, graph, solver, enumerator.minimumIndex, minimumBodyIndex + 1);
                //Console.WriteLine($"Swapping {enumerator.minimumIndex} with {minimumBodyIndex + 1}");
            }
            ++minimumBodyIndex;
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

    }
}
