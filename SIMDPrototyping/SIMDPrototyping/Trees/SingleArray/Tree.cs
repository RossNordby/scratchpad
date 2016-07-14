//#define OUTPUT
//#define NODE8

using BEPUutilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;




namespace SIMDPrototyping.Trees.SingleArray
{

    public struct Leaf
    {
        /// <summary>
        /// Identifier of the leaf.
        /// </summary>
        public int Id;
        /// <summary>
        /// Which node within the level the leaf is in.
        /// </summary>
        public int NodeIndex;
        /// <summary>
        /// Which child within the node the leaf is in.
        /// </summary>
        public int ChildIndex;
    }
    public unsafe partial class Tree : IDisposable
    {
        public const int ChildrenCapacity =
#if NODE32
            32;
#elif NODE16
            16;
#elif NODE8
            8;
#elif NODE4
            4;
#elif NODE2
            2;
#endif
        public Node[] Nodes;
        GCHandle nodesHandle;
        Node* nodes;
        int nodeCount;

        int AllocateNode(out bool pointersInvalidated)
        {
            if (nodeCount == NodeCapacity)
            {
                NodeCapacity *= 2;
                pointersInvalidated = true;
            }
            else
            {
                pointersInvalidated = false;
            }
            return nodeCount++;
        }


        object nodeLocker = new object();
        int AllocateNodeLocking(out bool pointersInvalidated)
        {
            lock (nodeLocker)
            {
                if (nodeCount == NodeCapacity)
                {
                    NodeCapacity *= 2;
                    pointersInvalidated = true;
                }
                else
                {
                    pointersInvalidated = false;
                }
                return nodeCount++;
            }
        }



        /// <summary>
        /// Gets or sets the space available for leaves in the tree.
        /// Setting this property invalidates any pointers to leaves.
        /// </summary>
        public int LeafCapacity
        {
            get
            {
                return Leaves.Length;
            }
            set
            {
                if (value < LeafCount)
                {
                    throw new ArgumentException("Cannot set the capacity to a value smaller than the current leaf count.");
                }
                Debug.Assert(leavesHandle.IsAllocated);
                leavesHandle.Free();
                var newLeaves = new Leaf[value];
                Array.Copy(Leaves, newLeaves, Leaves.Length);
                Leaves = newLeaves;
                leavesHandle = GCHandle.Alloc(Leaves, GCHandleType.Pinned);
                leaves = (Leaf*)nodesHandle.AddrOfPinnedObject();
            }
        }

        /// <summary>
        /// Gets or sets the space available for nodes in the tree.
        /// Setting this property invalidates any pointers to nodes.
        /// </summary>
        private int NodeCapacity
        {
            get

            {
                return Nodes.Length;

            }
            set
            {
                if (value < nodeCount)
                {
                    throw new ArgumentException("Cannot set the capacity to a value smaller than the current leaf count.");
                }
                Debug.Assert(nodesHandle.IsAllocated);
                nodesHandle.Free();
                var newNodes = new Node[Nodes.Length * 2];
                Array.Copy(Nodes, newNodes, Nodes.Length);
                Nodes = newNodes;
                nodesHandle = GCHandle.Alloc(Nodes, GCHandleType.Pinned);
                nodes = (Node*)nodesHandle.AddrOfPinnedObject();
            }
        }



        //Pointerized leaves don't really affect much. It just gets rid of the occasional bounds check, but that wasn't 
        //anywhere close to a bottleneck before. The ability to index into children of nodes is far more important.
        public Leaf[] Leaves;
        GCHandle leavesHandle;
        Leaf* leaves;
        int leafCount;
        public int LeafCount
        {
            get
            {
                return leafCount;
            }
        }

        public int NodeCount
        {
            get
            {
                return nodeCount;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AddLeaf(int id, int nodeIndex, int childIndex, out bool leavesInvalidated)
        {
            if (leafCount == Leaves.Length)
            {
                LeafCapacity *= 2;
                leavesInvalidated = true;
            }
            else
            {
                leavesInvalidated = false;
            }
            var leaf = leaves + leafCount;
            leaf->Id = id;
            leaf->NodeIndex = nodeIndex;
            leaf->ChildIndex = childIndex;
            return leafCount++;
        }


        /// <summary>
        /// Constructs an empty tree.
        /// </summary>
        /// <param name="initialLeafCapacity">Initial number of leaves to allocate room for.</param>
        public unsafe Tree(int initialLeafCapacity = 4096)
        {
            if (initialLeafCapacity <= 0)
                throw new ArgumentException("Initial leaf capacity must be positive.");

            //The number of nodes in the worst case could be equivalent to a binary tree.
            Nodes = new Node[Math.Max(1, initialLeafCapacity - 1)];
            nodesHandle = GCHandle.Alloc(Nodes, GCHandleType.Pinned);
            nodes = (Node*)nodesHandle.AddrOfPinnedObject();

            //The root always exists, even if there are no children in it. Makes certain bookkeeping simpler.
            nodeCount = 1;
            nodes->Parent = -1;
            nodes->IndexInParent = -1;

            Leaves = new Leaf[initialLeafCapacity];
            leavesHandle = GCHandle.Alloc(Leaves, GCHandleType.Pinned);
            leaves = (Leaf*)leavesHandle.AddrOfPinnedObject();
        }

        /// <summary>
        /// Resets the tree to a fresh post-construction state, clearing out leaves and nodes.
        /// </summary>
        public void Reset()
        {
            Array.Clear(Nodes, 0, nodeCount);
            Array.Clear(Leaves, 0, leafCount);
            leafCount = 0;
            //The root always exists, even if there are no children in it. Makes certain bookkeeping simpler.
            nodeCount = 1;
            nodes->Parent = -1;
            nodes->IndexInParent = -1;
        }

        /// <summary>
        /// Constructs a tree directly from provided leaves and nodes with no copying or validation.
        /// </summary>
        /// <param name="leaves">Leaves to use in the tree.</param>
        /// <param name="nodes">Nodes to use in the tree.</param>
        /// <param name="leafCount">Number of leaves in the tree. If negative, the length of the leaves array is used.</param>
        /// <param name="nodeCount">Number of nodes in the tree. If negative, the length of the nodes array is used.</param>
        public unsafe Tree(Leaf[] leaves, Node[] nodes, int leafCount = -1, int nodeCount = -1)
        {
            this.Leaves = leaves;
            this.Nodes = nodes;
            this.nodesHandle = GCHandle.Alloc(Nodes, GCHandleType.Pinned);
            this.nodes = (Node*)nodesHandle.AddrOfPinnedObject();
            this.leavesHandle = GCHandle.Alloc(Leaves, GCHandleType.Pinned);
            this.leaves = (Leaf*)leavesHandle.AddrOfPinnedObject();
            if (leafCount < 0)
                this.leafCount = leaves.Length;
            if (nodeCount < 0)
                this.nodeCount = nodes.Length;

        }
        

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int Encode(int index)
        {
            return -(index + 1);
        }











        public void Dispose()
        {
            if (nodesHandle.IsAllocated)
            {
                nodesHandle.Free();
                Debug.Assert(leavesHandle.IsAllocated, "Nodes and leaves should not be separately pinnable.");
                leavesHandle.Free();
            }
        }

#if DEBUG
        //TODO: may want to make this more permissive in the future; most users don't want to think about pinning and unpinning.
        ~Tree()
        {
            Debug.Assert(!nodesHandle.IsAllocated && !leavesHandle.IsAllocated, "The tree should be unpinned prior to finalization of the tree.");
        }
#endif

    }

}
