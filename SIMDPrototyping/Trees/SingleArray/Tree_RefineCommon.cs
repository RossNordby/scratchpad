using BEPUutilities.DataStructures;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    unsafe struct SubtreePriorityQueue
    {
        public int* Indices;
        public float* Costs;
        public int Count;

        public SubtreePriorityQueue(int* indices, float* costs)
        {
            this.Indices = indices;
            this.Costs = costs;
            Count = 0;
        }

        public int Pop()
        {
            Debug.Assert(Count > 0);
            return Indices[--Count];
        }

        public void Insert(Node* node, Node* nodes)
        {
            var sortedChildren = stackalloc int[node->ChildCount];
            var childCosts = stackalloc float[node->ChildCount];
            int internalChildrenCount = 0;
            var children = &node->ChildA;
            var bounds = &node->A;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    sortedChildren[internalChildrenCount] = children[i];
                    childCosts[internalChildrenCount] = Tree.ComputeBoundsMetric(ref bounds[i]);
                    ++internalChildrenCount;
                }
            }
            //Sort the children to prepare for batched insertion.
            //The priority queue's sort is from lowest to highest.
            //The new children will start at the highest end and move in, pushing everything outward.
            //That means the first child in this list should be the highest one.
            //Sort from high to low.
            //Use insertion sort due to extremely small sizes.
            for (int i = 1; i < internalChildrenCount; ++i)
            {
                var index = i;
                var previousIndex = i - 1;
                do
                {
                    if (childCosts[previousIndex] < childCosts[index])
                    {
                        var tempIndex = sortedChildren[index];
                        var tempCost = childCosts[index];
                        sortedChildren[index] = sortedChildren[previousIndex];
                        childCosts[index] = childCosts[previousIndex];
                        sortedChildren[previousIndex] = tempIndex;
                        childCosts[previousIndex] = tempCost;
                    }
                } while (previousIndex >= 0);
            }

            //Start at the end of the list and work your way back.
            int remainingChildrenCount = internalChildrenCount;
            int nextChildToAllocate = 0;
            //Comparison index can validly start at -1.
            //Consider: if Count is 0 and 4 children are being added,
            //the largest child should go in -1 + 4 = 3.
            int baseIndex = Count - 1;
            while (remainingChildrenCount > 0)
            {
                if (baseIndex >= 0 && Costs[baseIndex] > childCosts[nextChildToAllocate])
                {
                    //This subtree is still larger than the largest child.
                    //Move the subtree enough to make room for the remaining children.
                    var target = baseIndex + remainingChildrenCount;
                    Costs[target] = Costs[baseIndex];
                    Indices[target] = Indices[baseIndex];
                    --baseIndex;
                }
                else
                {
                    //Either this child is larger than the compared subtree, or there are no more subtrees to compare against.
                    var target = baseIndex + remainingChildrenCount;
                    Costs[target] = childCosts[nextChildToAllocate];
                    Indices[target] = sortedChildren[nextChildToAllocate];
                    --remainingChildrenCount;
                    ++nextChildToAllocate;
                }
            }
           
            Count += internalChildrenCount;


        }
    }
    partial class Tree
    {

        unsafe void CollectSubtrees2(int nodeIndex, int maximumSubtrees, ref QuickList<int> subtrees, ref QuickList<int> internalNodes, out float treeletCost)
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

            //Note that the treelet root's cost is excluded from the treeletCost.
            //That's because the treelet root cannot change.
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

                            var candidateCost = ComputeBoundsMetric(ref (&nodes[subtreeNode->Parent].A)[subtreeNode->IndexInParent]);
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

            //Note that the treelet root's cost is excluded from the treeletCost.
            //That's because the treelet root cannot change.
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

                            var candidateCost = ComputeBoundsMetric(ref (&nodes[subtreeNode->Parent].A)[subtreeNode->IndexInParent]);
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

    }
}
