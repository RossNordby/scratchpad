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

        unsafe void OptimizeGroupDFS(Node* optimizedNodes, Leaf* optimizedLeaves, ref int optimizedNodeCount, ref int optimizedLeafCount)
        {
            OptimizeGroupDFS(
        }
        unsafe void OptimizeGroupDFS(int optimizedNodeIndex, int optimizedParentNodeIndex, int nodeIndex, Node* optimizedNodes, Leaf* optimizedLeaves, ref int optimizedNodeCount, ref int optimizedLeafCount)
        {
            var node = nodes + nodeIndex;

            var optimizedNode = optimizedNodes + optimizedNodeIndex;
            *optimizedNode = *node;
            optimizedNode->Parent = optimizedParentNodeIndex;
            var optimizedChildren = &optimizedNode->ChildA;
            var nodeChildren = &node->ChildA;

            for (int i = 0; i < node->ChildCount; ++i)
            {

                if (nodeChildren[i] >= 0)
                {
                    optimizedChildren[i] = optimizedNodeCount++;

                }
                else
                {
                    var leafIndex = Encode(nodeChildren[i]);
                    var optimizedLeafIndex = optimizedLeafCount++;
                    var optimizedLeaf = optimizedLeaves + optimizedLeafIndex;
                    optimizedLeaf->Id = leaves[leafIndex].Id;
                    optimizedLeaf->NodeIndex = optimizedNodeIndex;
                    optimizedLeaf->ChildIndex = i;
                    optimizedChildren[i] = Encode(optimizedLeafIndex);

                }
            }
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (nodeChildren[i] >= 0)
                {
                    OptimizeGroupDFS(optimizedChildren[i], optimizedNodeIndex, nodeChildren[i], optimizedNodes, optimizedLeaves, ref optimizedNodeCount, ref optimizedLeafCount);
                }
            }
            return optimizedNodeIndex;
        }

        /// <summary>
        /// Creates a cache optimized version of the tree in the given leaves and nodes arrays.
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
        /// Creates a cache optimized copy of the tree that uses the minimum amount of memory possible.
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
