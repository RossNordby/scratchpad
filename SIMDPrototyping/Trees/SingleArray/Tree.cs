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
        Node[] NodesArray;
        GCHandle NodesHandle;
        Node* Nodes;
        int NodeCount;

        int Add(ref Node node)
        {
            if (NodeCount == NodesArray.Length)
            {
                Debug.Assert(NodesHandle.IsAllocated);
                NodesHandle.Free();
                var newNodes = new Node[NodesArray.Length * 2];
                Array.Copy(NodesArray, newNodes, NodesArray.Length);
                NodesArray = newNodes;
                NodesHandle = GCHandle.Alloc(NodesArray, GCHandleType.Pinned);
                Nodes = (Node*)NodesHandle.AddrOfPinnedObject();
            }
            NodesArray[NodeCount] = node;
            return NodeCount++;
        }




        public Leaf[] Leaves;
        int leafCount;

        public int LeafCount
        {
            get
            {
                return leafCount;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int AddLeaf(int id, int nodeIndex, int childIndex)
        {
            if (leafCount == Leaves.Length)
            {
                var newLeaves = new Leaf[leafCount * 2];
                Array.Copy(Leaves, newLeaves, leafCount);
                Leaves = newLeaves;
            }
            Leaves[leafCount].Id = id;
            Leaves[leafCount].NodeIndex = nodeIndex;
            Leaves[LeafCount].ChildIndex = childIndex;
            return leafCount++;
        }




        Vector<int>[] masks;


        public unsafe Tree(int initialLeafCapacity = 4096, int initialTreeDepth = 8)
        {
            if (initialTreeDepth <= 0)
                throw new ArgumentException("Initial tree depth must be positive.");
            if (initialLeafCapacity <= 0)
                throw new ArgumentException("Initial leaf capacity must be positive.");

            //The number of nodes in the worst case could be equivalent to a binary tree.
            NodesArray = new Node[Math.Max(1, initialLeafCapacity * 2 - 1)];
            NodesHandle = GCHandle.Alloc(NodesArray, GCHandleType.Pinned);
            Nodes = (Node*)NodesHandle.AddrOfPinnedObject();

            //The root always exists, even if there are no children in it. Makes certain bookkeeping simpler.
            NodeCount = 1;
            Nodes->Parent = -1;
            Nodes->IndexInParent = -1;

            Leaves = new Leaf[initialLeafCapacity];
            masks = Helpers.CreateMasks();
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
            if (NodesHandle.IsAllocated)
                NodesHandle.Free();
        }

#if DEBUG
        ~Tree()
        {
            for (int i = 0; i < Levels.Length; ++i)
            {
                Debug.Assert(!Levels[i].NodesHandle.IsAllocated, "No handle should still be allocated when the tree is finalized; implies a memory leak.");
            }
        }
#endif

    }

}
