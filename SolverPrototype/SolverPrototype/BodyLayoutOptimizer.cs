﻿using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System.Runtime.CompilerServices;
using static SolverPrototype.ConstraintConnectivityGraph;
using System;
using System.Diagnostics;

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
        public BodyLayoutOptimizer(Bodies bodies, ConstraintConnectivityGraph graph, Solver solver)
        {
            this.bodies = bodies;
            this.graph = graph;
            this.solver = solver;

            partialIslandDFSEnumerator = new PartialIslandDFSEnumerator
            {
                traversalStack = new QuickList<int>(new PassthroughBufferPool<int>()),
                bodies = bodies,
                graph = graph,
                solver = solver,
                maximumBodiesToVisit = 64
            };
        }
        struct SwapConstraintEnumerator : IForEach<BodyConstraintReference>
        {
            public Solver Solver;
            public int NewLocation;
            public void LoopBody(BodyConstraintReference constraint)
            {
                Solver.UpdateForBodyMemoryMove(constraint.ConnectingConstraintHandle, constraint.BodyIndexInConstraint, NewLocation);
            }
        }
        static void SwapBodyLocation(Bodies bodies, ConstraintConnectivityGraph graph, Solver solver, int oldLocation, int newLocation)
        {
            //Enumerate the body's current set of constraints, changing the reference in each to the new location. 
            SwapConstraintEnumerator enumerator;
            enumerator.Solver = solver;
            enumerator.NewLocation = newLocation;
            graph.EnumerateConstraints(oldLocation, ref enumerator);

            //Update the body and graph locations.
            bodies.Swap(oldLocation, newLocation);
            graph.SwapBodies(oldLocation, newLocation);
        }

        int dumbBodyIndex = 0;

        struct DumbIncrementalEnumerator : IForEachRef<ConnectedBody>
        {
            public Bodies bodies;
            public ConstraintConnectivityGraph graph;
            public Solver solver;
            public int slotIndex;
            public void LoopBody(ref ConnectedBody connection)
            {
                //Only pull bodies over that are to the right. This helps limit pointless fighting.
                //With this condition, objects within an island will tend to move towards the position of the leftmost body.
                //Without it, any progress towards island-level convergence could be undone by the next iteration.
                if (connection.BodyIndex > slotIndex)
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
                    SwapBodyLocation(bodies, graph, solver, connection.BodyIndex, newLocation);
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

        struct PartialIslandDFSEnumerator : IForEachRef<ConnectedBody>
        {
            public Bodies bodies;
            public ConstraintConnectivityGraph graph;
            public Solver solver;
            //We effectively do a full traversal over multiple frames. We have to store the stack for this to work.
            public QuickList<int> traversalStack;
            //The target index is just the last recorded exclusive endpoint of the island. In other words, it's the place where the next island body will be put.
            public int targetIndex;
            public int currentBodyIndex;
            public int maximumBodiesToVisit;
            public int visitedBodyCount;
            public void LoopBody(ref ConnectedBody connection)
            {
                //Should this node be swapped into position? 
                if (connection.BodyIndex > targetIndex)
                {
                    //Note that we update the memory location immediately. This could affect the next loop iteration.
                    //But this is fine; the next iteration will load from that modified data and everything will remain consistent.
                    var newLocation = targetIndex++;
                    Debug.Assert(newLocation > currentBodyIndex, "The target index should always progress ahead of the traversal. Did something get reset incorrectly?");
                    SwapBodyLocation(bodies, graph, solver, connection.BodyIndex, newLocation);
                    //Note that we mark the new location for traversal, since it was moved.
                    traversalStack.Add(newLocation);
                }
                else if (connection.BodyIndex > currentBodyIndex)
                {
                    //While this body should not be swapped because it has already been swapped by an earlier traversal visit, 
                    //it is still a candidate for continued traversal. It might have children that cannot be reached by other paths.
                    traversalStack.Add(connection.BodyIndex);
                }
            }
        }
        PartialIslandDFSEnumerator partialIslandDFSEnumerator;

        public void PartialIslandOptimizeDFS()
        {
            //With the observation that the full island DFS traversal is a decent cache optimization heuristic, attempt to do the same thing except spread over multiple frames.
            //This can clearly get invalidated by changes to the topology between frames, but temporary suboptimality will not cause correctness problems.

            //A few important implementation notes:
            //1) The target index advances with every swap performed.
            //2) "Visited" bodies are not explicitly tracked. Instead, the traversal will refuse to visit any bodies to the left of the current body.
            //Further, no swaps will occur with any body to the left of the target index.
            //Any body before the target index has already been swapped into position by an earlier traversal (heuristically speaking).
            //3) Swaps are performed inline, eliminating the need for any temporary body storage (or long-term locks in the multithreaded implementation).
            partialIslandDFSEnumerator.visitedBodyCount = 0;
            //First, attempt to continue any previous traversals.
            do
            {
                if (partialIslandDFSEnumerator.targetIndex >= bodies.BodyCount)
                {
                    //The target index has walked outside the bounds of the body set. While we could wrap around and continue, that would a different heuristic
                    //for swapping and visitation- currently we use 'to the right' which isn't well defined on a ring.
                    //Instead, for simplicity's sake, the target simply resets to the first index, and the traversal stack gets cleared.
                    partialIslandDFSEnumerator.targetIndex = 0;
                    partialIslandDFSEnumerator.traversalStack.Count = 0;
                }
                if (partialIslandDFSEnumerator.traversalStack.Count == 0)
                {
                    //There is no active traversal. Start one.
                    partialIslandDFSEnumerator.traversalStack.Add(partialIslandDFSEnumerator.targetIndex++);
                }
                partialIslandDFSEnumerator.traversalStack.Pop(out partialIslandDFSEnumerator.currentBodyIndex);
                //It's possible for the target index to fall behind if no swaps are needed. 
                partialIslandDFSEnumerator.targetIndex = Math.Max(partialIslandDFSEnumerator.targetIndex, partialIslandDFSEnumerator.currentBodyIndex + 1);
                graph.EnumerateConnectedBodies(partialIslandDFSEnumerator.currentBodyIndex, ref partialIslandDFSEnumerator);
                ++partialIslandDFSEnumerator.visitedBodyCount;
            } while (partialIslandDFSEnumerator.visitedBodyCount < partialIslandDFSEnumerator.maximumBodiesToVisit);
        }

        public void IslandOptimizeDFS()
        {
            //Simply grab an entire island by walking through the connection graph.
            //This trivially guarantees island locality and a stable depth first traversal order.
            //Beating this in terms of cache quality would likely require some pretty complex heuristics (root choice, branching choice...),
            //but the traversal and swap time is an issue. Very large islands are not rare. 
            //This is doubly true because multithreading this would be a pain (likely with little gain).

            //On the upside, with a good choice of bodies, this will converge pretty quickly.


        }

        public void IslandOptimizeBFS()
        {
            //Same idea as the DFS version. This is just a sanity test; it should be significantly worse.
            //1) While BFS can guarantee the contiguity of all children (and all bodies at that traversal distance), the solver operates on batches that
            //do not include the same body twice. In other words, while BFS produces a decent result if the child-parent pairs were iterated in order,
            //the solver is explicitly not doing that.
            //2) Apart from the root and its first child, we never guarantee that any direct connections will be adjacent in memory. In fact, we make it harder by putting a bunch
            //of other nodes in between them. While this doesn't guarantee that there won't be ANY adjacency, it doesn't do any better than randomly shuffling the island.


        }

        //Note that external systems which have to respond to body movements will generally be okay without their own synchronization in multithreaded cache optimization.
        //For example, the constraint handle, type batch index, index in type batch, and body index in constraint do not change. If we properly synchronize
        //the body accesses, then the changes to constraint bodies will be synchronized too.

        //Of course, it may turn out that the overhead of the synchronization (a per body interlocked call) is so high relative to the actual 'work' being done that
        //a single threaded implementation running at the same time as some other compatible stage just vastly outperforms it. It is very likely that the multithreaded version
        //will be 2-6 times slower than the single threaded version per thread.

    }
}
