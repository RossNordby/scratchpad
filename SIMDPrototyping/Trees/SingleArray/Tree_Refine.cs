using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
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
            public BoundingBox BoundingBox;
            public int A;
            public int B;
            public int LeafCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static int Add(ref TempNode newNode, TempNode* nodes, ref int count)
            {
                nodes[count] = newNode;
                return count++;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void FastRemoveAt(int index, int* remainingNodes, ref int remainingNodesCount)
            {
                var lastIndex = remainingNodesCount - 1;
                remainingNodes[index] = remainingNodes[lastIndex];
                remainingNodesCount = lastIndex;
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
            //Note the size: we first put every possible subtree in, so subtrees.Count.
            //Then, we add up subtrees.Count - 1 internal nodes without removing earlier slots.
            TempNode* tempNodes = stackalloc TempNode[subtrees.Count * 2 - 1];
            int tempNodeCount = subtrees.Count;
            int* remainingNodes = stackalloc int[subtrees.Count];
            int remainingNodesCount = subtrees.Count;

            for (int i = 0; i < subtrees.Count; ++i)
            {
                var tempNode = tempNodes + i;
                tempNode->A = Encode(i);
                if (subtrees.Elements[i] >= 0)
                {
                    //It's an internal node, so look at the parent.
                    var subtreeNode = nodes + subtrees.Elements[i];
                    tempNode->BoundingBox = (&nodes[subtreeNode->Parent].A)[subtreeNode->IndexInParent];
                    tempNode->LeafCount = (&nodes[subtreeNode->Parent].LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node, so grab the bounding box from the owning node.
                    var leaf = leaves + Encode(subtrees.Elements[i]);
                    var parentNode = nodes + leaf->NodeIndex;
                    tempNode->BoundingBox = (&parentNode->A)[leaf->ChildIndex];
                    tempNode->LeafCount = 1;
                }

                //Add a reference to the remaining list.
                remainingNodes[i] = i;
            }

            while (remainingNodesCount >= 2)
            {
                //Determine which pair of subtrees has the smallest cost.
                //(Smallest absolute cost is used instead of *increase* in cost because absolute tends to move bigger objects up the tree, which is desirable.)
                float bestCost = 0;
                int bestA = 0, bestB = 0;
                for (int i = 0; i < remainingNodesCount; ++i)
                {
                    for (int j = i + 1; j < remainingNodesCount; ++j)
                    {
                        var nodeIndexA = remainingNodes[i];
                        var nodeIndexB = remainingNodes[j];
                        BoundingBox merged;
                        BoundingBox.Merge(ref tempNodes[nodeIndexA].BoundingBox, ref tempNodes[nodeIndexB].BoundingBox, out merged);
                        var cost = ComputeBoundsHeuristic(ref merged);
                        if (cost < bestCost)
                        {
                            bestCost = cost;
                            bestA = i;
                            bestB = j;
                        }
                    }
                }
                {
                    //Create a new temp node based on the best pair.
                    TempNode newTempNode;
                    newTempNode.A = remainingNodes[bestA];
                    newTempNode.B = remainingNodes[bestB];
                    //Remerging here may or may not be faster than repeatedly caching 'best' candidates from above. It is a really, really cheap operation, after all, apart from cache issues.
                    BoundingBox.Merge(ref tempNodes[newTempNode.A].BoundingBox, ref tempNodes[newTempNode.B].BoundingBox, out newTempNode.BoundingBox);
                    newTempNode.LeafCount = tempNodes[newTempNode.A].LeafCount + tempNodes[newTempNode.B].LeafCount;

                    //Remove the best options from the list.
                    if (bestA > bestB)
                    {
                        TempNode.FastRemoveAt(bestA, remainingNodes, ref remainingNodesCount);
                        TempNode.FastRemoveAt(bestB, remainingNodes, ref remainingNodesCount);
                    }
                    else
                    {
                        TempNode.FastRemoveAt(bestB, remainingNodes, ref remainingNodesCount);
                        TempNode.FastRemoveAt(bestA, remainingNodes, ref remainingNodesCount);
                    }
                    //Add the reference to the new node.
                    var newIndex = TempNode.Add(ref newTempNode, tempNodes, ref tempNodeCount);
                    remainingNodes[remainingNodesCount++] = newIndex;
                }
            }

            //The 2-ary proto-treelet is ready.
            //Collapse it into an n-ary tree.
            const int collapseCount = ChildrenCapacity == 32 ? 4 : ChildrenCapacity == 16 ? 3 : ChildrenCapacity == 8 ? 2 : ChildrenCapacity == 4 ? 1 : 0;

            //Remember: All positive indices in the tempnodes array refer to other temp nodes: they are internal references. Encoded references point back to indices in the subtrees list.

            Debug.Assert(remainingNodesCount == 1);
            int parent = nodes[nodeIndex].Parent;
            int indexInParent = nodes[nodeIndex].IndexInParent;

            var reifiedIndex = BuildChild(parent, indexInParent, tempNodes, tempNodeCount - 1, collapseCount, ref subtrees, ref internalNodes);
            Debug.Assert((&nodes[parent].ChildA)[indexInParent] == reifiedIndex);

            if (internalNodes.Count > 0)
            {
                //There were some spare internal nodes left. Apparently, the tree was compressed a little bit.
                //Remove them from the real tree.
                //TODO: Multithreading issue.
                for (int i = 0; i < internalNodes.Count; ++i)
                {
                    RemoveNodeAt(internalNodes.Elements[i]);
                }
            }
            internalNodes.Dispose();
            subtrees.Dispose();
            

        }

        unsafe int BuildChild(int parent, int indexInParent, TempNode* tempNodes, int tempNodeIndex, int collapseCount, ref QuickList<int> subtrees, ref QuickList<int> internalNodes)
        {
            //Get ready to build a real node out of this.
            int internalNodeIndex;
            if (internalNodes.Count > 0)
            {
                //There is an old internal node that we can use.
                //Note that we remove from the beginning to guarantee that the root stays at index 0 in the real tree.
                //The internal nodes were gathered such that the first internal node is the root, and the first attempt to pull an internal
                //node will be the root of the treelet.
                internalNodeIndex = internalNodes.Elements[0];
                internalNodes.FastRemoveAt(0);
            }
            else
            {
                //There was no pre-existing internal node that we could use. Apparently, the tree gained some internal nodes.
                //TODO: Multithreading issue.
                internalNodeIndex = AllocateNode();
            }
            var internalNode = nodes + internalNodeIndex;
            internalNode->ChildCount = 0;
            internalNode->Parent = parent;
            internalNode->IndexInParent = indexInParent;
            int* internalNodeChildren = &internalNode->ChildA;
            CollapseTree(collapseCount, tempNodes, tempNodeIndex, internalNodeChildren, ref internalNode->ChildCount, &internalNode->A, &internalNode->LeafCountA);


            //The node now contains valid bounding boxes, but the children indices are not yet valid. They're pointing into the tempnodes.
            //Reify each one in sequence.
            for (int i = 0; i < internalNode->ChildCount; ++i)
            {
                if (internalNodeChildren[i] >= 0)
                {
                    internalNodeChildren[i] = BuildChild(internalNodeIndex, i, tempNodes, internalNodeChildren[i], collapseCount, ref subtrees, ref internalNodes);
                }
                else
                {
                    //It's a subtree. Reifying subtrees is really easy! Just take pull the pointers until you're pointing back at the real node.
                    internalNodeChildren[i] = subtrees.Elements[Encode(internalNodeChildren[i])];
                }
            }
            return internalNodeIndex;
        }

        unsafe void CollapseTree(int depthRemaining, TempNode* nodes, int nodeIndex, int* nodeChildren, ref int childCount, BoundingBox* nodeBounds, int* leafCounts)
        {
            var node = nodes + nodeIndex;
            if (node->A >= 0)
            {
                //Internal node.
                if (depthRemaining > 0)
                {
                    depthRemaining -= 1;
                    CollapseTree(depthRemaining, nodes, node->A, nodeChildren, ref childCount, nodeBounds, leafCounts);
                    CollapseTree(depthRemaining, nodes, node->B, nodeChildren, ref childCount, nodeBounds, leafCounts);
                }
                else
                {
                    //Reached the bottom of the recursion. Add the collected children.
                    int a = childCount++;
                    int b = childCount++;
                    nodeBounds[a] = nodes[node->A].BoundingBox;
                    nodeBounds[b] = nodes[node->B].BoundingBox;
                    nodeChildren[a] = node->A;
                    nodeChildren[b] = node->B;
                    leafCounts[a] = nodes[node->A].LeafCount;
                    leafCounts[b] = nodes[node->B].LeafCount;
                }

            }
            else
            {
                //Leaf node.
                var index = childCount++;
                nodeBounds[index] = node->BoundingBox;
                nodeChildren[index] = node->A;
                leafCounts[index] = 1;
            }
        }

        /// <summary>
        /// Executes one pass of bottom-up refinement.
        /// </summary>
        public unsafe void Refine()
        {

        }
    }
}
