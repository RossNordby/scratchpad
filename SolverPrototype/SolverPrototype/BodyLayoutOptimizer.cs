using BEPUutilities2.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    class ConstraintConnectivityGraph
    {
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
        struct List
        {
            public BufferRegion Region;
            public int Count;

            //Note that we don't technically constrain the type to be a struct. It's not even a sufficient condition for safety, since structs can contain references.
            //Rather than cluttering everything with a bunch of security theater, trust the user.
            //Who knows, maybe they really want to put a managed reference type in an array such that the GC can no longer track it!

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Add<T>(SuballocatedBufferPool bufferPool, ref List list, ref T item)
            {
                //TODO: verify that this is just a shift...
                if (list.Count == list.Region.GetLengthForType<T>())
                {
                    bufferPool.Resize(ref list.Region, list.Region.Power + 1);
                }
                Unsafe.Add(ref bufferPool.GetStart<T>(ref list.Region), list.Count) = item;
                ++list.Count;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Add<T>(SuballocatedBufferPool bufferPool, ref List list, T item)
            {
                //TODO: Confirm internal inlining.
                Add(bufferPool, ref list, ref item);
            }
            //Keeping with the quick-collection convention of 'fast' removes meaning that the last element is pulled into the removed slot, rather than preserving order.
            //Note that we're using an extremely non-idiomatic comparison scheme here- we require that a type parameter be provided that specifies a struct
            //implementor of the comparison interface. You could reasonably ask why:
            //1) IEquatable doesn't give you the option to specify different forms of equatability, and can run into annoyances when an external type doesn't support it.
            //2) Providing a comparison delegate incurs delegate call overhead.
            //3) Using a struct comparer forces JIT specialization.
            //This is basically working around a language expressiveness limitation.
            //A little bit questionable, but this is in large part just a test to see how it works out.

            //TODO: For larger types, it would also benefit from a ref parametered comparison. You could provide another overload that takes a ref item.
            //If the user cares enough to choose the ref overload, they almost certainly want to provide a ref comparer.

            //Also note that none of the removal functions bother to clear values to defaults.
            //That means pointers to reference types could remain in the array- but don't worry everything is fine, because the GC can't see those references anyway.
            //The backing array is byte[]. :) :) :         )

            /// <summary>
            /// Removes the first item from the list that is equal to the given item based on the specified comparer.
            /// </summary>
            /// <typeparam name="T">Type of the item to remove.</typeparam>
            /// <typeparam name="TComparer">Comparer used to determine whether a list element is equal to the item to remove..</typeparam>
            /// <param name="bufferPool">Buffer pool that the list was allocated from.</param>
            /// <param name="list">List to remove from.</param>
            /// <param name="item">Item to remove from the list.</param>
            /// <returns>True if the item was present and removed, false otherwise.</returns>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static bool FastRemove<T, TComparer>(SuballocatedBufferPool bufferPool, ref List list, T item) where TComparer : struct, IEqualityComparer<T>
            {
                var comparer = default(TComparer);
                ref var start = ref bufferPool.GetStart<T>(ref list.Region);
                for (int i = 0; i < list.Count; ++i)
                {
                    if (comparer.Equals(item, Unsafe.Add(ref start, i)))
                    {
                        FastRemoveAt<T>(bufferPool, ref list, i);
                        return true;
                    }
                }
                return false;
            }

            /// <summary>
            /// Removes an element at the given index.
            /// </summary>
            /// <typeparam name="T">Type of the removed element.</typeparam>
            /// <param name="bufferPool">Buffer pool that the list was allocated from.</param>
            /// <param name="list">List to remove from.</param>
            /// <param name="index">Index to remove in terms of elements of the specified type.</param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void FastRemoveAt<T>(SuballocatedBufferPool bufferPool, ref List list, int index)
            {
                Debug.Assert(index >= 0 && index < list.Count);
                --list.Count;
                if (index != list.Count)
                {
                    //Have to swap the last element into the removed slot.
                    ref var start = ref bufferPool.GetStart<T>(ref list.Region);
                    Unsafe.Add(ref start, index) = Unsafe.Add(ref start, list.Count);
                }
            }

        }
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
        List[] constraintLists;
        SuballocatedBufferPool bufferPool;

        public ConstraintConnectivityGraph(int initialBodyCountEstimate = 8192, int initialConstraintCountPerBodyEstimate = 8)
        {
            ConstraintCountPerBodyEstimate = initialConstraintCountPerBodyEstimate;
            var capacityInBytes = initialBodyCountEstimate * initialConstraintCountPerBodyEstimate * sizeof(int);
            var allocator = new Pow2Allocator(16, initialBodyCountEstimate);
            //TODO: It's often the case that sharing a pool like this among many users will reduce the overall memory use.
            //For now, for the sake of simplicity and guaranteeing thread access, the graph has its own allocator.
            //I suspect that there will be many uses for such a non-locked 'bookkeeping' buffer pool. 
            var bufferPool = new SuballocatedBufferPool(capacityInBytes, allocator);

            constraintLists = new List[initialBodyCountEstimate];
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
            ref var list = ref constraintLists[bodyIndex];
            bufferPool.Allocate(initialConstraintCountPerBodyPower, out list.Region);
            list.Count = 0;
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
            List.Add(bufferPool, ref constraintLists[bodyIndex], constraintHandle);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveConstraint(int bodyIndex, int constraintHandle)
        {
            List.FastRemove(bufferPool, ref constraintLists[bodyIndex], constraintHandle);
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
