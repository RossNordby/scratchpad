﻿using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;


namespace SolverPrototype.CollisionDetection
{
    public unsafe partial class Tree : IDisposable
    {
        public BufferPool Pool;
        public Buffer<Node> Nodes;
        //We cache a raw pointer for now. Buffer indexing isn't completely free yet. Also, this implementation was originally developed on raw pointers, so changing it would require effort.
        Node* nodes;
        int nodeCount;
        public int NodeCount
        {
            get
            {
                return nodeCount;
            }
        }

        //Pointerized leaves don't really affect much. It just gets rid of the occasional bounds check, but that wasn't 
        //anywhere close to a bottleneck before. The ability to index into children of nodes is far more important.
        public Buffer<Leaf> Leaves;
        Leaf* leaves;
        int leafCount;
        public int LeafCount
        {
            get
            {
                return leafCount;
            }
        }
        public IdPool<Buffer<int>, BufferPool<int>> LeafSlotsPool;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AllocateNode()
        {
            Debug.Assert(Nodes.Length > nodeCount,
                "Any attempt to allocate a node should not overrun the allocated nodes. For all operations that allocate nodes, capacity should be preallocated.");
            return nodeCount++;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AddLeaf(int nodeIndex, int childIndex)
        {
            Debug.Assert(leafCount < Leaves.Length,
                "Any attempt to allocate a leaf should not overrun the allocated leaves. For all operations that allocate leaves, capacity should be preallocated.");
            var leaf = leaves + leafCount;
            *leaf = new Leaf(nodeIndex, childIndex);
            //While we do track the leaf count for convenience purposes, there is no guarantee 
            ++leafCount;
            return LeafSlotsPool.Take();
        }


        /// <summary>
        /// Constructs an empty tree.
        /// </summary>
        /// <param name="initialLeafCapacity">Initial number of leaves to allocate room for.</param>
        public unsafe Tree(BufferPool pool, int initialLeafCapacity = 4096)
        {
            if (initialLeafCapacity <= 0)
                throw new ArgumentException("Initial leaf capacity must be positive.");

            Pool = pool;
            LeafSlotsPool = new IdPool<Buffer<int>, BufferPool<int>>(pool.SpecializeFor<int>());
            Resize(initialLeafCapacity);

        }

        //TODO: Could use a constructor or factory that can make it easy to take deserialized tree data without having to rerun a builder or hack with the backing memory.


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int Encode(int index)
        {
            return -1 - index;
        }


        //TODO: EnsureCapacity and Compact

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InitializeRoot()
        {
            //The root always exists, even if there are no children in it. Makes some bookkeeping simpler.
            nodeCount = 1;
            nodes->Parent = -1;
            nodes->IndexInParent = -1;
        }

        /// <summary>
        /// Resizes the buffers backing the tree's nodes and leaves. Will not shrink the buffers below the size needed by the currently resident nodes and leaves.
        /// </summary>
        /// <param name="targetLeafSlotCount">The desired number of available leaf slots.</param>
        public void Resize(int targetLeafSlotCount)
        {
            //Note that it's not safe to resize below the size of potentially used leaves. If the user wants to go smaller, they'll need to explicitly deal with the leaves somehow first.
            var leafCapacityForTarget = BufferPool<Leaf>.GetLowestContainingElementCount(Math.Max(LeafSlotsPool.HighestPossiblyClaimedId, targetLeafSlotCount));
            var nodeCapacityForTarget = BufferPool<Node>.GetLowestContainingElementCount(Math.Max(nodeCount, targetLeafSlotCount - 1));
            bool wasAllocated = Leaves.Allocated;
            Debug.Assert(Leaves.Allocated == Nodes.Allocated);
            if (leafCapacityForTarget != Leaves.Length)
            {
                Pool.SpecializeFor<Leaf>().Resize(ref Leaves, leafCapacityForTarget, LeafSlotsPool.HighestPossiblyClaimedId);
                leaves = (Leaf*)Leaves.Memory;
            }
            if (nodeCapacityForTarget != Nodes.Length)
            {
                Pool.SpecializeFor<Node>().Resize(ref Nodes, nodeCapacityForTarget, nodeCount);
                nodes = (Node*)Nodes.Memory;
            }
            if (!wasAllocated)
            {
                InitializeRoot();
            }
        }


        /// <summary>
        /// Resets the tree to a fresh post-construction state, clearing out leaves and nodes but leaving the backing resources intact.
        /// </summary>
        public void Clear()
        {
            leafCount = 0;
            InitializeRoot();
        }

        /// <summary>
        /// Disposes the tree's backing resources, returning them to the Pool currently associated with the tree.
        /// </summary>
        /// <remarks>Disposed trees can be reused if EnsureCapacity or Resize is used to rehydrate them.</remarks>
        public void Dispose()
        {
            Debug.Assert(Nodes.Allocated == Leaves.Allocated, "Nodes and leaves should have consistent lifetimes.");
            if (Nodes.Allocated)
            {
                Pool.SpecializeFor<Node>().Return(ref Nodes);
                Pool.SpecializeFor<Leaf>().Return(ref Leaves);
            }
        }

    }

}
