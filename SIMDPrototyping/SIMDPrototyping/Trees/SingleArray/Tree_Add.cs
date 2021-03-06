﻿//#define OUTPUT
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
    partial class Tree
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void MergeLeafNodes(int newLeafId, ref BoundingBox newLeafBounds, int parentIndex, int indexInParent, ref BoundingBox merged, out bool nodesInvalidated, out bool leavesInvalidated)
        {
            //It's a leaf node.
            //Create a new internal node with the new leaf and the old leaf as children.
            //this is the only place where a new level could potentially be created.

            var newNodeIndex = AllocateNode(out nodesInvalidated);
            var newNode = nodes + newNodeIndex;
            newNode->ChildCount = 2;
            newNode->Parent = parentIndex;
            newNode->IndexInParent = indexInParent;
            //The first child of the new node is the old leaf. Insert its bounding box.
            var parentNode = nodes + parentIndex;
            newNode->A = (&parentNode->A)[indexInParent];
            newNode->ChildA = (&parentNode->ChildA)[indexInParent];
            newNode->LeafCountA = 1;

            //Insert the new leaf into the second child slot.
            newNode->B = newLeafBounds;
            newNode->LeafCountB = 1;
            var leafIndex = AddLeaf(newLeafId, newNodeIndex, 1, out leavesInvalidated);
            nodes[newNodeIndex].ChildB = Encode(leafIndex);

            //Update the old leaf node with the new index information.
            var oldLeafIndex = Encode(newNode->ChildA);
            leaves[oldLeafIndex].NodeIndex = newNodeIndex;
            leaves[oldLeafIndex].ChildIndex = 0;

            //Update the original node's child pointer and bounding box.
            (&parentNode->ChildA)[indexInParent] = newNodeIndex;
            (&parentNode->A)[indexInParent] = merged;
            ++(&parentNode->LeafCountA)[indexInParent];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void InsertLeafIntoEmptySlot(int leafId, ref BoundingBox leafBox, int nodeIndex, int childIndex, Node* node, out bool leavesInvalidated)
        {
            ++node->ChildCount;
            var leafIndex = AddLeaf(leafId, nodeIndex, childIndex, out leavesInvalidated);
            (&node->ChildA)[childIndex] = Encode(leafIndex);
            (&node->A)[childIndex] = leafBox;
            (&node->LeafCountA)[childIndex] = 1;
        }
        enum BestInsertionChoice
        {
            EmptySlot,
            MergeLeaf,
            Internal
        }
        public unsafe void Add(int leafId, ref BoundingBox box)
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            int nodeIndex = 0;
#if OUTPUT
            List<int> choices = new List<int>();
#endif
            while (true)
            {
                //Which child should the leaf belong to?

                //Give the leaf to whichever node had the least volume change. 
                var node = nodes + nodeIndex;
                var boundingBoxes = &node->A;
                var children = &node->ChildA;
                var leafCounts = &node->LeafCountA;
                int minimumIndex = 0;
                float minimumChange = float.MaxValue;
                BoundingBox merged = new BoundingBox();
                var bestChoice = BestInsertionChoice.EmptySlot;
                if (node->ChildCount < ChildrenCapacity)
                {
                    //The best slot will, at best, be tied with inserting it in a leaf node because the change in heuristic cost
                    //for filling an empty slot is zero.
                    minimumIndex = node->ChildCount;
                    merged = box;
                    minimumChange = 0;
                    bestChoice = BestInsertionChoice.EmptySlot;
                }
                else
                {
                    for (int i = 0; i < node->ChildCount; ++i)
                    {
                        BoundingBox mergedCandidate;
                        BoundingBox.Merge(ref boundingBoxes[i], ref box, out mergedCandidate);
                        var newCost = ComputeBoundsMetric(ref mergedCandidate);
                        //Since we already checked for an empty slot, the two remaining possibilities are merging with an existing leaf node
                        //and continuing down another internal node.
                        //Going into another internal node only increases the relevant cost (that of internal nodes) by the change in merged volume.
                        //Merging into a leaf node generates a whole new internal node, so it tends to be more expensive.
                        BestInsertionChoice choice;
                        float costChange;
                        if (children[i] >= 0)
                        {
                            choice = BestInsertionChoice.Internal;
                            costChange = newCost - ComputeBoundsMetric(ref boundingBoxes[i]);
                        }
                        else
                        {
                            choice = BestInsertionChoice.MergeLeaf;
                            costChange = newCost;
                        }


                        if (costChange < minimumChange)
                        {
                            minimumChange = costChange;
                            minimumIndex = i;
                            merged = mergedCandidate;
                            bestChoice = choice;
                        }

                    }

                }

#if OUTPUT
                Console.WriteLine($"Minimum index: {minimumIndex}, minimum volume increase: {minimum}");
                choices.Add(minimumIndex);
#endif
                switch (bestChoice)
                {
                    case BestInsertionChoice.EmptySlot:
                        {
                            //There is no child at all.
                            //Put the new leaf here.
                            bool leavesInvalidated;
                            InsertLeafIntoEmptySlot(leafId, ref box, nodeIndex, minimumIndex, node, out leavesInvalidated);
                            return;
                        }
                    case BestInsertionChoice.MergeLeaf:
                        {
                            bool nodesInvalidated, leavesInvalidated;
                            MergeLeafNodes(leafId, ref box, nodeIndex, minimumIndex, ref merged, out nodesInvalidated, out leavesInvalidated);
                            //No pointers need to be updated. All the old ones are done with.
                            return;
                        }
                    case BestInsertionChoice.Internal:
                        {
                            //It's an internal node. Traverse to the next node.
                            boundingBoxes[minimumIndex] = merged;
                            nodeIndex = children[minimumIndex];
                            ++leafCounts[minimumIndex];
                            break;
                        }

                }

            }
        }
    }
}
