using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        unsafe void SwapNodes(int a, int b)
        {
            var temp = nodes[b];
            nodes[b] = nodes[a];
            nodes[a] = temp;

            //Update the parents' child pointers.
            var nodeA = nodes + a;
            var nodeB = nodes + b;
            (&nodes[nodeA->Parent].ChildA)[nodeA->IndexInParent] = a;
            (&nodes[nodeB->Parent].ChildA)[nodeB->IndexInParent] = b;

            //Update the children's parent pointers.
            var aChildren = &nodeA->ChildA;
            for (int i = 0; i < nodeA->ChildCount; ++i)
            {
                if (aChildren[i] >= 0)
                {
                    nodes[aChildren[i]].Parent = a;
                }
                else
                {
                    leaves[Encode(aChildren[i])].NodeIndex = a;
                }
            }
            var bChildren = &nodeB->ChildA;
            for (int i = 0; i < nodeB->ChildCount; ++i)
            {
                if (bChildren[i] >= 0)
                {
                    nodes[bChildren[i]].Parent = b;
                }
                else
                {
                    leaves[Encode(bChildren[i])].NodeIndex = b;
                }
            }
            if (nodeA->ChildCount == 0 || nodeB->ChildCount == 0)
            {
                Console.WriteLine("asdF");
            }
        }
        unsafe void CacheOptimizeDFS(int nodeIndex, ref int count)
        {
            //Since we're doing this top down, we know the current node is in its final position.
            //(Either the parent already moved it into its proper position, or this is the root.)
            //So, the node pointer is safe.
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            var originalChildCount = node->ChildCount;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    if (nodeIndex == children[i])
                        Console.WriteLine("ASDF");
                    var targetIndex = count++;
                    if (nodeIndex == targetIndex)
                        Console.WriteLine("ASDF");
                    if (targetIndex >= nodeCount)
                        Console.WriteLine("asdF");
                    var original = children[i];
                    if (children[i] != targetIndex)
                    {
                        SwapNodes(children[i], targetIndex);

                        if (children[i] == original)
                            Console.WriteLine("ASDF");

                    }
                    if (children[i] != targetIndex)
                        Console.WriteLine("asdF");
                    if (originalChildCount != node->ChildCount)
                        Console.WriteLine("assdf");
                    if (children[i] <= nodeIndex)
                        Console.WriteLine("ASDF");
                    if (children[i] >= count)
                        Console.WriteLine("break");
                    CacheOptimizeDFS(children[i], ref count);
                    if (originalChildCount != node->ChildCount)
                        Console.WriteLine("assdf");
                }
            }
        }
        public void CacheOptimize2()
        {
            int count = 1;
            CacheOptimizeDFS(0, ref count);
        }


        unsafe int OptimizeDFS(int optimizedParentIndex, int nodeIndex, Node* optimizedNodes, Leaf* optimizedLeaves, ref int optimizedNodeCount, ref int optimizedLeafCount)
        {
            var node = nodes + nodeIndex;
            var optimizedNodeIndex = optimizedNodeCount++;
            var optimizedNode = optimizedNodes + optimizedNodeIndex;
            *optimizedNode = *node;
            optimizedNode->Parent = optimizedParentIndex;
            var children = &optimizedNode->ChildA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    children[i] = OptimizeDFS(optimizedNodeIndex, children[i], optimizedNodes, optimizedLeaves, ref optimizedNodeCount, ref optimizedLeafCount);
                }
                else
                {
                    var leafIndex = Encode(children[i]);
                    var optimizedLeafIndex = optimizedLeafCount++;
                    var optimizedLeaf = optimizedLeaves + optimizedLeafIndex;
                    optimizedLeaf->Id = leaves[leafIndex].Id;
                    optimizedLeaf->NodeIndex = optimizedNodeIndex;
                    optimizedLeaf->ChildIndex = i;
                    children[i] = Encode(optimizedLeafIndex);

                }
            }
            return optimizedNodeIndex;
        }

        /// <summary>
        /// Creates an optimized version of the tree in the given leaves and nodes arrays.
        /// </summary>
        /// <param name="optimizedLeavesArray">Array to fill with optimized leaves.</param>
        /// <param name="optimizedNodesArray">Array to fill with optimized nodes.</param>
        public unsafe void CreateOptimized(Leaf[] optimizedLeavesArray, Node[] optimizedNodesArray)
        {
            if (optimizedLeavesArray.Length < LeafCount)
                throw new ArgumentException("Leaves array must be able to contain all leaves in this tree.");
            if (optimizedLeavesArray.Length < nodeCount)
                throw new ArgumentException("Nodes array must be able to contain all nodes in this tree.");

            fixed (Leaf* optimizedLeaves = optimizedLeavesArray)
            fixed (Node* optimizedNodes = optimizedNodesArray)
            {
                int optimizedNodeCount = 0;
                int optimizedLeafCount = 0;
                OptimizeDFS(-1, 0, optimizedNodes, optimizedLeaves, ref optimizedNodeCount, ref optimizedLeafCount);
                Debug.Assert(optimizedNodeCount == nodeCount);
                Debug.Assert(optimizedLeafCount == leafCount);
            }
        }
        /// <summary>
        /// Creates a cache and memory optimized copy of the tree.
        /// </summary>
        /// <returns>Cache and memory optimized copy of the tree.</returns>
        public unsafe Tree CreateOptimized()
        {
            var optimizedLeavesArray = new Leaf[leafCount];
            var optimizedNodesArray = new Node[nodeCount];
            CreateOptimized(optimizedLeavesArray, optimizedNodesArray);
            return new Tree(optimizedLeavesArray, optimizedNodesArray);
        }
    }
}
