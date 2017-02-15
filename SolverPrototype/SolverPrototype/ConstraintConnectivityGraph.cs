using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
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
                initialConstraintCountPerBodyPower = BufferPool.GetPoolIndex(value * Unsafe.SizeOf<BodyConstraintReference>());
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
        Solver solver;

        //You could bitpack these two into 4 bytes, but the value of that is pretty darn questionable.
        public struct BodyConstraintReference
        {
            public int ConnectingConstraintHandle;
            public int BodyIndexInConstraint;
        }


        public ConstraintConnectivityGraph(Solver solver, int initialBodyCountEstimate = 8192, int initialConstraintCountPerBodyEstimate = 8)
        {
            this.solver = solver;
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
            if (bodyIndex >= constraintLists.Length)
            {
                //Not enough room for this body! Resize required.
                Array.Resize(ref constraintLists, 1 << BufferPool.GetPoolIndex(bodyIndex + 1));
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
        public void AddConstraint(int bodyIndex, int constraintHandle, int bodyIndexInConstraint)
        {
            SuballocatedList.Add(bufferPool, ref constraintLists[bodyIndex], constraintHandle);
        }
        
        struct RemovalPredicate : IPredicate<BodyConstraintReference>
        {
            public int ConstraintHandleToRemove;
            public bool Test(BodyConstraintReference item)
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
            SuballocatedList.FastRemove<BodyConstraintReference, RemovalPredicate>(bufferPool, ref list, ref predicate);
        }

        //TODO: It's likely that we'll eventually have something very similar to all of this per body list stuff for collision detection pairs. We'll worry about
        //de-duping that code later.

        /// <summary>
        /// The index of a connected body and the connection by which it was reached.
        /// </summary>
        public struct ConnectedBody
        {
            public int BodyIndex;
            public int ConnectingConstraintHandle;
            public int BodyIndexInConstraint;
        }

        //sooooooo idiomatic

        struct ConstraintBodiesEnumerator<TInnerEnumerator> : IForEach<int> where TInnerEnumerator : IForEachRef<ConnectedBody>
        {
            public TInnerEnumerator InnerEnumerator;
            public int SourceBodyIndex;
            public int ConstraintHandle;
            public int IndexInConstraint;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                if (SourceBodyIndex != connectedBodyIndex)
                {
                    ConnectedBody connectedBody;
                    connectedBody.BodyIndex = connectedBodyIndex;
                    connectedBody.ConnectingConstraintHandle = ConstraintHandle;
                    connectedBody.BodyIndexInConstraint = IndexInConstraint++;
                    InnerEnumerator.LoopBody(ref connectedBody);
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Reset()
            {
                IndexInConstraint = 0;
            }
        }

        /// <summary>
        /// Enumerates all the bodies connected to a given body.
        /// Bodies which are connected by more than one constraint will be reported multiple times alongside the connecting constraints.
        /// </summary>
        /// <typeparam name="TEnumerator">Type of the enumerator to execute on each connected body.</typeparam>
        /// <param name="bodyIndex">Index of the body to enumerate the connections of. This body will not appear in the set of enumerated bodies, even if it is connected to itself somehow.</param>
        /// <param name="enumerator">Enumerator instance to run on each connected body.</param>
        public void EnumerateConnectedBodies<TEnumerator>(int bodyIndex, ref TEnumerator enumerator) where TEnumerator : IForEachRef<ConnectedBody>
        {
            ref var list = ref constraintLists[bodyIndex];
            ref var start = ref bufferPool.GetStart<int>(ref list.Region);
            ConstraintBodiesEnumerator<TEnumerator> constraintBodiesEnumerator;
            constraintBodiesEnumerator.InnerEnumerator = enumerator;
            constraintBodiesEnumerator.SourceBodyIndex = bodyIndex;
            constraintBodiesEnumerator.IndexInConstraint = 0;

            for (int i = 0; i < list.Count; ++i)
            {
                constraintBodiesEnumerator.ConstraintHandle = Unsafe.Add(ref start, i);
                solver.EnumerateConnectedBodyIndices(constraintBodiesEnumerator.ConstraintHandle, ref constraintBodiesEnumerator);
                constraintBodiesEnumerator.Reset();
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
        public void EnumerateConstraints<TEnumerator>(int bodyIndex, ref TEnumerator enumerator) where TEnumerator : IForEach<BodyConstraintReference>
        {
            ref var list = ref constraintLists[bodyIndex];
            ref var start = ref bufferPool.GetStart<BodyConstraintReference>(ref list.Region);

            for (int i = 0; i < list.Count; ++i)
            {
                enumerator.LoopBody(Unsafe.Add(ref start, i));
            }
        }
    }
}
