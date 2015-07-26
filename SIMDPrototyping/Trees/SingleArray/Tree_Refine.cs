using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        /// <summary>
        /// Defines a subtree by pointing to a particular node.
        /// May also be a leaf node; in that case, the node index is encoded.
        /// </summary>
        struct Subtree
        {
            //We cache the bounding box and node locally so that we can freely manipulate information without worrying about corrupting the tree.
            //Does the implementation really care?
            public BoundingBox BoundingBox;
            public int NodeIndex;
        }

        struct PotentialSubtree
        {
            public int NodeIndex;
            public float Cost;
        }

        unsafe void CollectSubtrees(int nodeIndex, int maximumSubtrees, ref QuickList<int> subtrees, ref QuickList<int> internalNodes)
        {
            //Collect subtrees iteratively by choosing the highest surface area subtree repeatedly.
            //This collects every child of a given node at once- the set of subtrees must not include only SOME of the children of a node.

            //(You could lift this restriction and only take some nodes, but it would complicate things. You could not simply remove
            //the parent and add its children to go deeper; it would require doing some post-fixup on the results of the construction
            //or perhaps constraining the generation process to leave room for the unaffected nodes.)


            var node = nodes + nodeIndex;
            var bounds = &node->A;
            var children = &node->ChildA;
            Debug.Assert(maximumSubtrees >= node->ChildCount, "Can't only consider some of a node's children, but specified maximumSubtrees precludes the treelet root's children.");
            //All of treelet root's children are included immediately. (Follows from above requirement.)
            for (int i = 0; i < node->ChildCount; ++i)
            {
                subtrees.Add(children[i]);
            }
            internalNodes.Add(nodeIndex);
            while (subtrees.Count < maximumSubtrees)
            {
                //Find the largest subtree.
                float highestCost = float.MinValue;
                int highestIndex = -1;
                for (int subtreeIndex = subtrees.Count - 1; subtreeIndex >= 0; ++subtreeIndex)
                {
                    var subtreeNodeIndex = subtrees.Elements[subtreeIndex];
                    var subtreeNode = nodes + subtreeNodeIndex;
                    var subtreeChildren = &subtreeNode->ChildA;
                    var subtreeBounds = &subtreeNode->A;

                    for (int childIndex = 0; childIndex < subtreeNode->ChildCount; ++childIndex)
                    {
                        //Only consider internal nodes as potential expansions.
                        if (subtreeChildren[childIndex] >= 0 &&
                            (subtrees.Count - 1 + nodes[subtreeChildren[childIndex]].ChildCount) <= maximumSubtrees) //Make sure that the new node would fit (after we remove the expanded node).
                        {
                            var candidateCost = ComputeBoundsHeuristic(ref subtreeBounds[childIndex]);
                            if (candidateCost > highestCost)
                            {
                                highestIndex = subtreeIndex;
                            }
                        }
                    }
                }
                if (highestIndex >= 0)
                {
                    //Found a subtree to expand.
                    var expandedNodeIndex = subtrees.Elements[highestIndex];
                    internalNodes.Add(expandedNodeIndex);
                    subtrees.FastRemoveAt(highestIndex);
                    //Add all the children to the set of subtrees.
                    //This is safe because we pre-validated the number of children in the node.
                    var expandedNode = nodes + highestIndex;
                    var expandedNodeChildren = &expandedNode->ChildA;
                    for (int i = 0; i < expandedNode->ChildCount; ++i)
                    {
                        subtrees.Add(expandedNodeChildren[i]);
                    }
                }
                else
                {
                    //No further expansions are possible. Either every option was a leaf, or the remaining expansions are too large to fit in the maximumSubtrees.
                    break;
                }

            }
        }


        unsafe struct TempNode
        {
            public int A;
            public int B;
            public BoundingBox BoundingBox;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Add(TempNode* node, ref int count)
            {
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void FastRemoveAt(TempNode* node, ref int count)
            {
            }
        }
        

        /// <summary>
        /// Collects a limited set of subtrees hanging from the specified node and performs a local treelet rebuild using a bottom-up agglomerative approach.
        /// </summary>
        /// <param name="nodeIndex">Root of the refinement treelet.</param>
        public unsafe void AgglomerativeRefine(int nodeIndex)
        {
            var maximumSubtrees = ChildrenCapacity * ChildrenCapacity;
            var poolIndex = BufferPool<int>.GetPoolIndex(maximumSubtrees);
            var subtrees = new QuickList<int>(BufferPools<int>.Thread, poolIndex);
            var internalNodes = new QuickList<int>(BufferPools<int>.Thread, poolIndex);
            CollectSubtrees(nodeIndex, maximumSubtrees, ref subtrees, ref internalNodes);

            //We're going to create a little binary tree via agglomeration, and then we'll collapse it into an n-ary tree.
            TempNode* tempNodes = stackalloc TempNode[subtrees.Count - 1];
            int tempNodeCount;

            //Determine which pair of subtrees has the smallest cost.
            //(Smallest absolute cost is used instead of *increase* in cost because absolute tends to move bigger objects up the tree, which is desirable.)
            float bestCost = 0;
            BoundingBox bestMerged = new BoundingBox();
            int bestPairA = 0;
            int bestPairB = 0;
            for (int i = 0; i < subtrees.Count; ++i)
            {
                for (int j = i + 1; j < subtrees.Count; ++j)
                {
                    BoundingBox merged;
                    BoundingBox.Merge(ref subtrees.Elements[i].BoundingBox, ref subtrees.Elements[j].BoundingBox, out merged);
                    var cost = ComputeBoundsHeuristic(ref merged);
                    if (cost < bestCost)
                    {
                        bestCost = cost;
                        bestMerged = merged;
                        bestPairA = i;
                        bestPairB = j;
                    }
                }
            }
            if (bestPairA > bestPairB)
            {
                subtrees.FastRemoveAt(bestPairA);
                subtrees.FastRemoveAt(bestPairB);
            }
            else
            {
                subtrees.FastRemoveAt(bestPairB);
                subtrees.FastRemoveAt(bestPairA);
            }
            //Add the new combined node.

            subtrees.Add(Encode(
        }

        /// <summary>
        /// Executes one pass of bottom-up refinement.
        /// </summary>
        public unsafe void Refine()
        {

        }
    }
}
