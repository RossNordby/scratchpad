﻿using BEPUutilities.DataStructures;
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

        unsafe void CollectSubtrees(int nodeIndex, int maximumSubtrees, ref QuickList<int> subtrees, ref QuickList<int> internalNodes, out float treeletCost)
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

            //Cache the index of the treelet root. Later, the root will be moved to the end of the list to guarantee that it's the first node that's used.
            //This provides the guarantee that the treelet root index will not change.
            var rootIndex = internalNodes.Count;
            internalNodes.Add(nodeIndex);

            treeletCost = 0;
            while (subtrees.Count < maximumSubtrees)
            {
                //Find the largest subtree.
                float highestCost = float.MinValue;
                int highestIndex = -1;
                for (int subtreeIndex = subtrees.Count - 1; subtreeIndex >= 0; --subtreeIndex)
                {
                    var subtreeNodeIndex = subtrees.Elements[subtreeIndex];
                    if (subtreeNodeIndex >= 0) //Only consider internal nodes.
                    {

                        var subtreeNode = nodes + subtreeNodeIndex;
                        if ((subtrees.Count - 1 + subtreeNode->ChildCount) <= maximumSubtrees) //Make sure that the new node would fit (after we remove the expanded node).
                        {

                            var candidateCost = ComputeBoundsHeuristic(ref (&nodes[subtreeNode->Parent].A)[subtreeNode->IndexInParent]);
                            if (candidateCost > highestCost)
                            {
                                highestIndex = subtreeIndex;
                                highestCost = candidateCost;
                            }
                        }
                    }
                }
                if (highestIndex >= 0)
                {
                    treeletCost += highestCost;
                    //Found a subtree to expand.
                    var expandedNodeIndex = subtrees.Elements[highestIndex];

                    internalNodes.Add(expandedNodeIndex);

                    subtrees.FastRemoveAt(highestIndex);
                    //Add all the children to the set of subtrees.
                    //This is safe because we pre-validated the number of children in the node.
                    var expandedNode = nodes + expandedNodeIndex;
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
            //Swap the treelet root into the last position so that the first internal node consumed is guaranteed to be the root.
            var lastIndex = internalNodes.Count - 1;
            var temp = internalNodes.Elements[lastIndex];
            internalNodes.Elements[lastIndex] = internalNodes.Elements[rootIndex];
            internalNodes.Elements[rootIndex] = temp;
        }


        unsafe struct TempNode
        {
            public BoundingBox BoundingBox;
            public int A;
            public int B;
            public int LeafCount;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static int Add(ref TempNode newNode, TempNode* tempNodes, ref int count)
            {
                tempNodes[count] = newNode;
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
        /// <param name="nodesInvalidated">True if the refinement process invalidated node pointers, false otherwise.</param>
        public unsafe void AgglomerativeRefine(int nodeIndex, ref QuickList<int> internalNodes, out bool nodesInvalidated)
        {
            var maximumSubtrees = ChildrenCapacity * ChildrenCapacity;
            var poolIndex = BufferPool<int>.GetPoolIndex(maximumSubtrees);
            var subtrees = new QuickList<int>(BufferPools<int>.Thread, poolIndex);
            float originalTreeletCost;
            int treeletInternalNodesStartIndex = internalNodes.Count;
            CollectSubtrees(nodeIndex, maximumSubtrees, ref subtrees, ref internalNodes, out originalTreeletCost);



            //We're going to create a little binary tree via agglomeration, and then we'll collapse it into an n-ary tree.
            //Note the size: we first put every possible subtree in, so subtrees.Count.
            //Then, we add up subtrees.Count - 1 internal nodes without removing earlier slots.
            int tempNodesCapacity = subtrees.Count * 2 - 1;

            var tempNodes = stackalloc TempNode[tempNodesCapacity];
            int tempNodeCount = subtrees.Count;
            int remainingNodesCapacity = subtrees.Count;

            var remainingNodes = stackalloc int[remainingNodesCapacity];
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
                    var leafIndex = Encode(subtrees.Elements[i]);
                    var leaf = leaves + leafIndex;
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
                float bestCost = float.MaxValue;
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
                    //BestA is always lower than bestB, so remove bestB first to avoid corrupting bestA index.
                    TempNode.FastRemoveAt(bestB, remainingNodes, ref remainingNodesCount);
                    TempNode.FastRemoveAt(bestA, remainingNodes, ref remainingNodesCount);

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

            var stagingNodeCapacity = maximumSubtrees - 1;
            var stagingNodes = stackalloc Node[maximumSubtrees - 1];
            int stagingNodeCount = 0;
            float newTreeletCost;
            var stagingRootIndex = BuildStagingChild(parent, indexInParent, tempNodes, tempNodeCount - 1, collapseCount, stagingNodes, ref stagingNodeCount, out newTreeletCost);
            Debug.Assert(stagingNodeCount < stagingNodeCapacity);

            if (newTreeletCost < originalTreeletCost)
            {
                //The refinement is an actual improvement.
                //Apply the staged nodes to real nodes!
                var reifiedIndex = ReifyStagingNode(parent, indexInParent, stagingNodes, 0, stagingNodeCapacity, ref subtrees, ref internalNodes, out nodesInvalidated);
                Debug.Assert(parent != -1 ? (&nodes[parent].ChildA)[indexInParent] == reifiedIndex : true, "The parent should agree with the child about the relationship.");
            }
            else
            {
                //The internal nodes collected by the most recent iteration of CollectSubtrees weren't replaced! Get them out of the pool.
                //TODO: Would be nice to do this in a slightly less gross way.
                for (int i = internalNodes.Count - 1; i >= treeletInternalNodesStartIndex; --i)
                {
                    internalNodes.FastRemoveAt(i);
                }
                nodesInvalidated = false;
            }

            //var reifiedIndex = BuildChild(parent, indexInParent, tempNodes, tempNodeCount - 1, tempNodeCount, collapseCount, ref subtrees, ref internalNodes, out nodesInvalidated);

            //Debug.Assert(parent != -1 ? (&nodes[parent].ChildA)[indexInParent] == reifiedIndex : true, "The parent should agree with the child about the relationship.");


            subtrees.Dispose();


        }

        unsafe int BuildStagingChild(int parent, int indexInParent, TempNode* tempNodes, int tempNodeIndex, int collapseCount,
            Node* stagingNodes, ref int stagingNodeCount, out float treeletCost)
        {
            //Get ready to build a staging node out of this.

            var internalNodeIndex = stagingNodeCount++;
            var internalNode = stagingNodes + internalNodeIndex;
            internalNode->Parent = parent;
            internalNode->IndexInParent = indexInParent;
            internalNode->ChildCount = 0;


            var internalNodeChildren = &internalNode->ChildA;
            var internalNodeBounds = &internalNode->A;

            CollapseTree(collapseCount, tempNodes, tempNodeIndex, internalNodeChildren, ref internalNode->ChildCount, internalNodeBounds, &internalNode->LeafCountA);

            treeletCost = 0;
            //ValidateLeafNodeIndices();

            //The node now contains valid bounding boxes, but the children indices are not yet valid. They're pointing into the tempnodes.
            //Reify each one in sequence.
            for (int i = 0; i < internalNode->ChildCount; ++i)
            {
                treeletCost += ComputeBoundsHeuristic(ref internalNodeBounds[i]);
                //The internalNodeChildren[i] can be negative, as a result of a subtree being encountered in CollapseTree.
                //tempNodes[internalNodeChildren[i]].A is never negative for any internal node (internalNodeChildren[i] >= 0),
                //because we pre-collapsed that reference in CollapseTree for convenience.

                if (internalNodeChildren[i] >= 0)
                {
                    //It's still an internal node.
                    float childTreeletCost;
                    var childIndex = BuildStagingChild(internalNodeIndex, i, tempNodes, internalNodeChildren[i], collapseCount, stagingNodes, ref stagingNodeCount, out childTreeletCost);
                    treeletCost += childTreeletCost;

                    internalNodeChildren[i] = childIndex;

                }
                //If it's a subtree, then we don't need to change it at all.
            }
            return internalNodeIndex;
        }



        unsafe void CollapseTree(int depthRemaining, TempNode* tempNodes, int tempNodeIndex, int* nodeChildren, ref int childCount, BoundingBox* nodeBounds, int* leafCounts)
        {

            var tempNode = tempNodes + tempNodeIndex;
            if (tempNode->A >= 0)
            {

                //Internal node.
                if (depthRemaining > 0)
                {
                    depthRemaining -= 1;
                    CollapseTree(depthRemaining, tempNodes, tempNode->A, nodeChildren, ref childCount, nodeBounds, leafCounts);
                    CollapseTree(depthRemaining, tempNodes, tempNode->B, nodeChildren, ref childCount, nodeBounds, leafCounts);
                }
                else
                {
                    //Reached the bottom of the recursion. Add the collected children.
                    int a = childCount++;
                    int b = childCount++;

                    nodeBounds[a] = tempNodes[tempNode->A].BoundingBox;
                    nodeBounds[b] = tempNodes[tempNode->B].BoundingBox;
                    //If the child is a leaf, collapse it. Slightly less annoying to handle it here than later.
                    //This is a byproduct of the awkward data layout for tempnodes...
                    if (tempNodes[tempNode->A].A >= 0)
                        nodeChildren[a] = tempNode->A;
                    else
                        nodeChildren[a] = tempNodes[tempNode->A].A; //It's a leaf.
                    if (tempNodes[tempNode->B].A >= 0)
                        nodeChildren[b] = tempNode->B;
                    else
                        nodeChildren[b] = tempNodes[tempNode->B].A; //It's a leaf. Leaf index is always stored in A...


                    leafCounts[a] = tempNodes[tempNode->A].LeafCount;
                    leafCounts[b] = tempNodes[tempNode->B].LeafCount;


                }

            }
            else
            {
                //Leaf node.
                var index = childCount++;
                nodeBounds[index] = tempNode->BoundingBox;
                nodeChildren[index] = tempNode->A;
                leafCounts[index] = tempNode->LeafCount;
            }
        }

        unsafe int ReifyStagingNode(int parent, int indexInParent, Node* stagingNodes, int stagingNodeIndex, int stagingNodeCapacity, ref QuickList<int> subtrees, ref QuickList<int> internalNodes, out bool nodesInvalidated)
        {

            nodesInvalidated = false;
            int internalNodeIndex;
            if (internalNodes.Count > 0)
            {
                //There is an old internal node that we can use.
                //Note that we remove from the end to guarantee that the treelet root does not change location.
                //The CollectSubtrees function guarantees that the treelet root is placed at the very end of the spare internalNodes list.
                var lastIndex = internalNodes.Count - 1;
                internalNodeIndex = internalNodes.Elements[lastIndex];
                internalNodes.FastRemoveAt(lastIndex);
            }
            else
            {
                //There was no pre-existing internal node that we could use. Apparently, the tree gained some internal nodes.
                //TODO: Multithreading issue.
                //Watch out, calls to AllocateNode potentially invalidate all extant node pointers.
                //Fortunately, there are no pointers to invalidate... here. Propagate it up.
                bool addCausedInvalidation;
                internalNodeIndex = AllocateNode(out addCausedInvalidation);
                if (addCausedInvalidation)
                    nodesInvalidated = true;
            }

            //To make the staging node real, it requires an accurate parent pointer, index in parent, and child indices.
            //Copy the staging node into the real tree.
            var stagingNode = stagingNodes + stagingNodeIndex;
            var internalNode = nodes + internalNodeIndex;
            *internalNode = *stagingNode;
            internalNode->Parent = parent;
            internalNode->IndexInParent = indexInParent;
            var internalNodeChildren = &internalNode->ChildA;
            for (int i = 0; i < internalNode->ChildCount; ++i)
            {
                if (internalNodeChildren[i] >= 0)
                {
                    bool childNodesInvalidated;
                    internalNodeChildren[i] = ReifyStagingNode(internalNodeIndex, i, stagingNodes, internalNodeChildren[i], stagingNodeCapacity, ref subtrees, ref internalNodes, out childNodesInvalidated);
                    if (childNodesInvalidated)
                    {
                        internalNode = nodes + internalNodeIndex;
                        internalNodeChildren = &internalNode->ChildA;
                        nodesInvalidated = true;
                    }
                }
                else
                {
                    //It's a subtree. Update its pointers.
                    var subtreeIndex = subtrees.Elements[Encode(internalNodeChildren[i])];
                    internalNodeChildren[i] = subtreeIndex;
                    if (subtreeIndex >= 0)
                    {
                        Debug.Assert(subtreeIndex >= 0 && subtreeIndex < nodeCount);
                        //Subtree is an internal node. Update its parent pointers.
                        nodes[subtreeIndex].IndexInParent = i;
                        nodes[subtreeIndex].Parent = internalNodeIndex;

                    }
                    else
                    {
                        //Subtree is a leaf node. Update its parent pointers.
                        var leafIndex = Encode(subtreeIndex);
                        Debug.Assert(leafIndex >= 0 && leafIndex < LeafCount);
                        var leaf = leaves + leafIndex;
                        leaf->ChildIndex = i;
                        leaf->NodeIndex = internalNodeIndex;
                    }
                }
            }
            return internalNodeIndex;
        }

        void RemoveUnusedInternalNodes(ref QuickList<int> spareNodes)
        {
            if (spareNodes.Count > 0)
            {
                //There were some spare internal nodes left. Apparently, the tree was compressed a little bit.
                //Remove them from the real tree.
                //TODO: Multithreading issue.
                //Remove highest to lowest to avoid any situation where removing could invalidate an index.
                Array.Sort(spareNodes.Elements, 0, spareNodes.Count);
                for (int i = spareNodes.Count - 1; i >= 0; --i)
                {
                    RemoveNodeAt(spareNodes.Elements[i]);
                }
            }
        }

        unsafe void TryToBottomUpRefine(int[] refinementFlags, int nodeIndex, ref QuickList<int> spareInternalNodes)
        {
            if (++refinementFlags[nodeIndex] == nodes[nodeIndex].ChildCount)
            {
                bool nodesInvalidated;
                AgglomerativeRefine(nodeIndex, ref spareInternalNodes, out nodesInvalidated);

                var parent = nodes[nodeIndex].Parent;
                if (parent != -1)
                {
                    TryToBottomUpRefine(refinementFlags, parent, ref spareInternalNodes);
                }
            }
        }


        /// <summary>
        /// Executes one pass of bottom-up refinement.
        /// </summary>
        public unsafe void BottomUpRefine()
        {
            //If this works out, should probably choose a more efficient flagging approach.
            //Note the size: it needs to contain all possible internal nodes.
            //TODO: This is actually bugged, because the refinement flags do not update if the nodes move.
            //And the nodes CAN move.
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
            var refinementFlags = new int[leafCount * 2 - 1];
            for (int i = 0; i < nodeCount; ++i)
            {
                refinementFlags[i] = 0;
            }
            for (int i = 0; i < leafCount; ++i)
            {
                TryToBottomUpRefine(refinementFlags, leaves[i].NodeIndex, ref spareNodes);
                //Validate();
            }
            //Console.WriteLine($"root children: {nodes->ChildCount}");
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }



        private unsafe void TopDownRefine(int nodeIndex, ref QuickList<int> spareNodes)
        {
            bool nodesInvalidated;
            AgglomerativeRefine(nodeIndex, ref spareNodes, out nodesInvalidated);
            //The root of the tree is guaranteed to stay in position, so nodeIndex is still valid.

            //Node pointers can be invalidated, so don't hold a reference between executions.
            for (int i = 0; i < nodes[nodeIndex].ChildCount; ++i)
            {
                var child = (&nodes[nodeIndex].ChildA)[i];
                if (child >= 0)
                {
                    TopDownRefine(child, ref spareNodes);

                }
            }


        }
        public unsafe void TopDownRefine()
        {
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
            TopDownRefine(0, ref spareNodes);
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }


    }
}