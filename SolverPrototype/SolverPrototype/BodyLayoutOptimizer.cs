using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System.Runtime.CompilerServices;
using static SolverPrototype.ConstraintConnectivityGraph;
using System;

namespace SolverPrototype
{

    struct BodyConstraintEntry
    {

    }
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
                    bodies.Swap(connection.BodyIndex, newLocation);
                    //Note that graph.EnumerateConnectedBodies explicitly excludes the body whose constraints we are enumerating, 
                    //so we don't have to worry about having the rug pulled by this list swap.
                    //(Also, !(x > x) for many values of x.)
                    graph.SwapBodies(connection.BodyIndex, newLocation);
                    solver.UpdateForBodyMemoryMove(connection.ConnectingConstraintHandle, connection.BodyIndexInConstraint, newLocation);
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

        public void LazyIncrementalOptimize()
        {
            //This is similar to the dumb variant, but checks for parent-child agreement on positioning before swapping.
            //At a given body, it looks through all bodies that it is connected to that are positioned to the left, looking for a body that it can call a parent.
            //Another body is a parent when its first neighbor to the right in memory is the current body.
            //Note that it is possible for a body to not have a 'parent' in this sense; 
            //consider an island composed of single central body with many neighbors that do not connect to each other. 
            //Only one of those neighbors will position itself next to the central body in memory, leaving the others at arbitrary locations in memory.
            //So, while this approach converges in terms of parent-child relationships, it will not guarantee locality of an entire island.
            //The main value here is in avoiding the unnecessary swaps of the dumb variant.
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
        //TODO: Partial island optimization. True island optimization is probably not a good idea because islands can be arbitrarily large. However, there is value in the 
        //extra guarantees it provides. You can compromise and achieve independence from island size by simply doing a partial island traversal. Traversing in simple DFS order
        //until a number of bodies are accumulated. You should be able to get away even with fairly large maximum sizes- 64, 128... So only very large islands would suffer at all.

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
