//#define OUTPUT
//#define NODE8

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
        Node[] nodesArray;
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


        /// <summary>
        /// Gets or sets the space available for leaves in the tree.
        /// Setting this property invalidates any pointers to leaves.
        /// </summary>
        public int LeafCapacity
        {
            get
            {
                return LeavesArray.Length;
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
                Array.Copy(LeavesArray, newLeaves, LeavesArray.Length);
                LeavesArray = newLeaves;
                leavesHandle = GCHandle.Alloc(LeavesArray, GCHandleType.Pinned);
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
                return nodesArray.Length;

            }
            set
            {
                if (value < nodeCount)
                {
                    throw new ArgumentException("Cannot set the capacity to a value smaller than the current leaf count.");
                }
                Debug.Assert(nodesHandle.IsAllocated);
                nodesHandle.Free();
                var newNodes = new Node[nodesArray.Length * 2];
                Array.Copy(nodesArray, newNodes, nodesArray.Length);
                nodesArray = newNodes;
                nodesHandle = GCHandle.Alloc(nodesArray, GCHandleType.Pinned);
                nodes = (Node*)nodesHandle.AddrOfPinnedObject();
            }
        }



        //Pointerized leaves don't really affect much. It just gets rid of the occasional bounds check, but that wasn't 
        //anywhere close to a bottleneck before. The ability to index into children of nodes is far more important.
        public Leaf[] LeavesArray;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AddLeaf(int id, int nodeIndex, int childIndex, out bool leavesInvalidated)
        {
            if (leafCount == LeavesArray.Length)
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


        public unsafe void GetLeafBoundingBox(int i, out BoundingBox boundingBox)
        {
            boundingBox = (&nodes[leaves[i].NodeIndex].A)[leaves[i].ChildIndex];
        }


        public unsafe Tree(int initialLeafCapacity = 4096, int initialTreeDepth = 8)
        {
            if (initialTreeDepth <= 0)
                throw new ArgumentException("Initial tree depth must be positive.");
            if (initialLeafCapacity <= 0)
                throw new ArgumentException("Initial leaf capacity must be positive.");

            //The number of nodes in the worst case could be equivalent to a binary tree.
            nodesArray = new Node[Math.Max(1, initialLeafCapacity * 2 - 1)];
            nodesHandle = GCHandle.Alloc(nodesArray, GCHandleType.Pinned);
            nodes = (Node*)nodesHandle.AddrOfPinnedObject();

            //The root always exists, even if there are no children in it. Makes certain bookkeeping simpler.
            nodeCount = 1;
            nodes->Parent = -1;
            nodes->IndexInParent = -1;

            LeavesArray = new Leaf[initialLeafCapacity];
            leavesHandle = GCHandle.Alloc(LeavesArray, GCHandleType.Pinned);
            leaves = (Leaf*)leavesHandle.AddrOfPinnedObject();
        }

        //Node initialNode;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void InitializeNode(out Node node)
        {
            node = new Node();
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
