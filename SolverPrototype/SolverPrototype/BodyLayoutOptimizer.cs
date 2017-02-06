using BEPUutilities2.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    class ConstraintConnectivityGraph
    {
        int initialConstraintCountPerBodyPower;
        /// <summary>
        /// Gets or sets the expected number of constraints associated with each body.
        /// Note that this should be a power of 2; if a non-power of 2 is set, it will be interpreted as the next higher power of 2.
        /// </summary>
        public int ConstraintCountPerBodyEstimate
        {
            get
            {
                return 1 << initialConstraintCountPerBodyPower;
            }
            set
            {
                initialConstraintCountPerBodyPower = BufferPool.GetPoolIndex(value);
            }
        }

        //We need a per-body list of constraints because any movement of a body requires updating the connected constraints.
        //Since we have that, we'll also use it to find adjacent bodies. Finding adjacent bodies is required for two purposes:
        //1) Deactivating an island of low energy bodies.
        //2) Optimizing the memory layout of bodies so that connected bodies tend to be adjacent or at least nearby.

        //Conceptually, we could do something simple like a List<List<int>> for a per-body list of constraint handles.
        //An obvious optimization to that is to pool the internal lists to avoid GC overhead, but there is another factor to consider beyond just garbage generation-
        //heap complexity.

        //The goal here is to reduce the cost of a GC. While the engine should never produce garbage itself during regular execution, the application using it might.
        //The cost of a GC is partially related to the number of references it has to track down. An array of reference types requires examining every one of those references.
        //So, if you have 16384 bodies with lists stored in QuickList<int>[] representation, the GC has at least 32768 references it needs to consider!
        //In contrast, if you pack the representation into a single int[] with the lists suballocated from it, the GC doesn't need to scan it- ints aren't reference types.

        //So we could use a custom allocator (like, say, the BEPUutilities Allocator), but there's a much simpler way that is no more wasteful than the QuickList representation 
        //in terms of memory use- preallocated arrays that you can pull fixed size blocks out of. 

        //We'll store references to these memory blocks and treat them like the arrays we'd use for a QuickList. This will be a little ugly due to the typelessness
        //of the backing array, but hopefully in the future we can unify things a bit more.
        SuballocatedList[] constraintLists;
        SuballocatedBufferPool bufferPool;

        public ConstraintConnectivityGraph(int initialBodyCountEstimate = 8192, int initialConstraintCountPerBodyEstimate = 8)
        {
            ConstraintCountPerBodyEstimate = initialConstraintCountPerBodyEstimate;
            var capacityInBytes = initialBodyCountEstimate * initialConstraintCountPerBodyEstimate * sizeof(int);
            var allocator = new Pow2Allocator(16, initialBodyCountEstimate);
            //TODO: It's often the case that sharing a pool like this among many users will reduce the overall memory use.
            //For now, for the sake of simplicity and guaranteeing thread access, the graph has its own allocator.
            //I suspect that there will be many uses for such a non-locked 'bookkeeping' buffer pool. 
            bufferPool = new SuballocatedBufferPool(capacityInBytes, allocator);

            constraintLists = new SuballocatedList[initialBodyCountEstimate];
        }

        //Note that constraints only contain direct references to the memory locations of bodies, not to their handles.
        //While we could get the handles from the memory locations, it is cheaper/simpler just to deal with the memory locations directly.
        //To this end, the constraint lists are ordered according to the body memory order, not the handle order.
        //Every time a body moves due to cache optimization or removal, these lists must be updated.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SwapBodies(int oldLocation, int newLocation)
        {
            var temp = constraintLists[oldLocation];
            constraintLists[oldLocation] = constraintLists[newLocation];
            constraintLists[newLocation] = temp;
        }

        /// <summary>
        /// Adds a list for a body to the given location.
        /// </summary>
        /// <param name="bodyIndex">Location to allocate a list.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddBodyList(int bodyIndex)
        {
            //Note that we trust the user to provide valid locations. The graph shouldn't do any of its own positioning- it is slaved to the body memory layout.
            //This isn't a system that external users will be using under any normal circumstance, so trust should be okay.
            if (constraintLists.Length > bodyIndex)
            {
                //Not enough room for this body! Resize required.
                Array.Resize(ref constraintLists, BufferPool.GetPoolIndex(bodyIndex));
            }
            SuballocatedList.Create(bufferPool, initialConstraintCountPerBodyPower, out constraintLists[bodyIndex]);
        }

        /// <summary>
        /// Frees the list associated with a body at the given location.
        /// </summary>
        /// <param name="bodyIndex">Location to remove.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveBodyList(int bodyIndex)
        {
            bufferPool.Free(ref constraintLists[bodyIndex].Region);
        }

        /// <summary>
        /// Adds a constraint to the specified body.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to add the constraint handle to.</param>
        /// <param name="constraintHandle">Constraint handle to add to the body's list.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddConstraint(int bodyIndex, int constraintHandle)
        {
            SuballocatedList.Add(bufferPool, ref constraintLists[bodyIndex], constraintHandle);
        }

        struct IntEqualityComparer : IEqualityComparer<int>
        {
            ///TODO: Does the JIT pay attention to this inline? It should be able to due to type specialization, but it is a corner case; double check.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Equals(int x, int y)
            {
                return x == y;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetHashCode(int i)
            {
                return i.GetHashCode();
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveConstraint(int bodyIndex, int constraintHandle)
        { 
            //This uses a linear search. That's fine; bodies will rarely have more than a handful of constraints associated with them.
            //Attempting to use something like a hash set for fast removes would just introduce more constant overhead and slow it down on average.
            ref var list = ref constraintLists[bodyIndex];
            SuballocatedList.FastRemove<int, IntEqualityComparer>(bufferPool, ref list, constraintHandle);
        }

        //TODO: It's likely that we'll eventually have something very similar to all of this per body list stuff for collision detection pairs. We'll worry about
        //de-duping that code later.
    }
    /// <summary>
    /// Incrementally changes the layout of a set of bodies to minimize the cache misses associated with the solver and other systems that rely on connection following.
    /// </summary>
    class BodyLayoutOptimizer
    {
        Bodies bodies;
        ConstraintConnectivityGraph graph;
        public BodyLayoutOptimizer(Bodies bodies, ConstraintConnectivityGraph graph)
        {
            this.bodies = bodies;
            this.graph = graph;
        }
    }
}
