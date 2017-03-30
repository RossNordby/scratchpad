﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public class ConstraintConnectivityGraph
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
                initialConstraintCountPerBodyPower = SpanHelper.GetContainingPowerOf2(value);
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
        //So, if you have 16384 bodies with lists stored in List<int>[] representation, the GC has at least 32768 references it needs to consider (the list, and its internal array)!
        //In contrast, if you pack the representation into a single int[] with the lists suballocated from it, the GC doesn't need to scan it- ints aren't reference types.

        //QuickLists based on a pointer-backed Buffer<int> contain no references at all, so QuickList<int, Buffer<int>>[] only costs a single reference- for the top level array.
        //The rest of it is just a bunch of value types.
        QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>[] constraintLists;
        BufferPool<BodyConstraintReference> bufferPool;
        Solver solver;

        //You could bitpack these two into 4 bytes, but the value of that is pretty darn questionable.
        public struct BodyConstraintReference
        {
            public int ConnectingConstraintHandle;
            public int BodyIndexInConstraint;
        }


        /// <summary>
        /// Constructs a constraint connectivity graph.
        /// </summary>
        /// <param name="solver">Solver associated with the constraints in this graph.</param>
        /// <param name="rawPool">Pool from which per-body lists are allocated.</param>
        /// <param name="initialBodyCountEstimate">Initial estimate for the number of bodies which will exist in the graph.
        /// If the number is exceeded, an internal buffer will resize and produce garbage.</param>
        /// <param name="initialConstraintCountPerBodyEstimate">Initial estimate for the number of constraints that will exist for each body.
        /// If the number is exceeded, the body list will be resized, but the old list will be returned to the pool for reuse.</param>
        public ConstraintConnectivityGraph(Solver solver, BufferPool rawPool, int initialBodyCountEstimate = 8192, int initialConstraintCountPerBodyEstimate = 8)
        {
            this.solver = solver;
            ConstraintCountPerBodyEstimate = initialConstraintCountPerBodyEstimate;
            var capacityInBytes = initialBodyCountEstimate * initialConstraintCountPerBodyEstimate * sizeof(int);
            bufferPool = rawPool.SpecializeFor<BodyConstraintReference>();

            constraintLists = new QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>[initialBodyCountEstimate];
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
            if (bodyIndex >= constraintLists.Length)
            {
                //Not enough room for this body! Resize required.
                Array.Resize(ref constraintLists, 1 << SpanHelper.GetContainingPowerOf2(bodyIndex + 1));
            }
            QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>>.Create(bufferPool, initialConstraintCountPerBodyPower, out constraintLists[bodyIndex]);
        }

        /// <summary>
        /// Frees the list associated with a body at the given location and returns true if the freed list was empty.
        /// </summary>
        /// <param name="bodyIndex">Location to remove.</param>
        /// <param name="replacementIndex">If nonnegative, the index to pull a replacement list from.</param>
        /// <returns>True if the freed list was empty, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveBodyList(int bodyIndex, int replacementIndex)
        {
            ref var list = ref constraintLists[bodyIndex];
            var empty = list.Count == 0;
            list.Dispose(bufferPool);

            if (replacementIndex >= 0)
            {
                constraintLists[bodyIndex] = constraintLists[replacementIndex];
            }
            return empty;
        }

        /// <summary>
        /// Adds a constraint to the specified body.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to add the constraint handle to.</param>
        /// <param name="constraintHandle">Constraint handle to add to the body's list.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddConstraint(int bodyIndex, int constraintHandle, int bodyIndexInConstraint)
        {
            BodyConstraintReference constraint;
            constraint.ConnectingConstraintHandle = constraintHandle;
            constraint.BodyIndexInConstraint = bodyIndexInConstraint;
            constraintLists[bodyIndex].Add(ref constraint, bufferPool);
        }

        struct RemovalPredicate : IPredicate<BodyConstraintReference>
        {
            public int ConstraintHandleToRemove;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Matches(ref BodyConstraintReference item)
            {
                return item.ConnectingConstraintHandle == ConstraintHandleToRemove;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveConstraint(int bodyIndex, int constraintHandle)
        {
            //This uses a linear search. That's fine; bodies will rarely have more than a handful of constraints associated with them.
            //Attempting to use something like a hash set for fast removes would just introduce more constant overhead and slow it down on average.
            ref var list = ref constraintLists[bodyIndex];
            RemovalPredicate predicate;
            predicate.ConstraintHandleToRemove = constraintHandle;
            list.FastRemove(ref predicate);
        }

        //TODO: It's likely that we'll eventually have something very similar to all of this per body list stuff for collision detection pairs. We'll worry about
        //de-duping that code later.


        //sooooooo idiomatic

        struct ConstraintBodiesEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEach<int>
        {
            public TInnerEnumerator InnerEnumerator;
            public int SourceBodyIndex;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                if (SourceBodyIndex != connectedBodyIndex)
                {
                    //Note that this may report the same body multiple times if it is connected multiple times! That's fine and potentially useful; let the user deal with it.
                    InnerEnumerator.LoopBody(connectedBodyIndex);
                }
            }

        }

        /// <summary>
        /// Enumerates all the bodies connected to a given body.
        /// Bodies which are connected by more than one constraint will be reported multiple times.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to execute on each connected body.</typeparam>
        /// <param name="bodyIndex">Index of the body to enumerate the connections of. This body will not appear in the set of enumerated bodies, even if it is connected to itself somehow.</param>
        /// <param name="enumerator">Enumerator instance to run on each connected body.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnumerateConnectedBodies<TEnumerator>(int bodyIndex, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var list = ref constraintLists[bodyIndex];
            ConstraintBodiesEnumerator<TEnumerator> constraintBodiesEnumerator;
            constraintBodiesEnumerator.InnerEnumerator = enumerator;
            constraintBodiesEnumerator.SourceBodyIndex = bodyIndex;

            //Note reverse iteration. This is useful when performing O(1) removals where the last element is put into the position of the removed element.
            //Non-reversed iteration would result in skipped elements if the loop body removed anything. This relies on convention; any remover should be aware of this order.
            for (int i = list.Count - 1; i >= 0; --i)
            {
                solver.EnumerateConnectedBodyIndices(list[i].ConnectingConstraintHandle, ref constraintBodiesEnumerator);
            }
            //Note that we have to assume the enumerator contains state mutated by the internal loop bodies.
            //If it's a value type, those mutations won't be reflected in the original reference. 
            //Copy them back in.
            enumerator = constraintBodiesEnumerator.InnerEnumerator;
        }

        /// <summary>
        /// Enumerates all the constraint handles connected to a given body.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to execute on each constraint handle.</typeparam>
        /// <param name="bodyIndex">Index of the body to enumerate the constraints of.</param>
        /// <param name="enumerator">Enumerator to execute on each constraint handle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnumerateConstraints<TEnumerator>(int bodyIndex, ref TEnumerator enumerator) where TEnumerator : IForEach<BodyConstraintReference>
        {
            ref var list = ref constraintLists[bodyIndex];
            //Note reverse iteration. This is useful when performing O(1) removals where the last element is put into the position of the removed element.
            //Non-reversed iteration would result in skipped elements if the loop body removed anything. This relies on convention; any remover should be aware of this order.
            for (int i = list.Count - 1; i >= 0; --i)
            {
                enumerator.LoopBody(list[i]);
            }
        }
        /// <summary>
        /// Checks whether a body is referenced by the given constraint handle.
        /// </summary>
        /// <param name="bodyIndex">Body to check for a constraint.</param>
        /// <param name="constraintHandle">Constraint handle to look for in the body's constraint list.</param>
        /// <returns>True if the body is connected to the constraint, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool BodyIsConstrainedBy(int bodyIndex, int constraintHandle)
        {
            //This is a special case that bypasses the IForEach enumerators. It's not strictly necessary, but it is pretty convenient.
            //It's unclear how valuable this actually is- this shouldn't be a particularly common operation. We actually only added it for debugging purposes.
            //The only functional benefit it has over the IForEach variants is that it will early out. But you could create an early-outing enumerator.
            ref var list = ref constraintLists[bodyIndex];
            
            for (int i = 0; i < list.Count; ++i)
            {
                if (list[i].ConnectingConstraintHandle == constraintHandle)
                    return true;
            }
            return false;
        }
    }
}
