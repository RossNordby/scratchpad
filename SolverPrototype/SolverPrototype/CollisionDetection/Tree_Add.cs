using BEPUutilities2;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        /// <summary>
        /// Merges a new leaf node with an existing leaf node, producing a new internal node referencing both leaves, and then returns the index of the leaf node.
        /// </summary>
        /// <param name="newLeafBounds">Bounding box of the leaf being added.</param>
        /// <param name="parentIndex">Index of the parent node that the existing leaf belongs to.</param>
        /// <param name="indexInParent">Index of the child wtihin the parent node that the existing leaf belongs to.</param>
        /// <param name="merged">Bounding box holding both the new and existing leaves.</param>
        /// <returns>Index of the leaf </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe int MergeLeafNodes(ref BoundingBox newLeafBounds, int parentIndex, int indexInParent, ref BoundingBox merged)
        {
            //It's a leaf node.
            //Create a new internal node with the new leaf and the old leaf as children.
            //this is the only place where a new level could potentially be created.

            var newNodeIndex = AllocateNode();
            var newNode = nodes + newNodeIndex;
            newNode->ChildCount = 2;
            newNode->Parent = parentIndex;
            newNode->IndexInParent = indexInParent;
            //The first child of the new node is the old leaf. Insert its bounding box.
            var parentNode = nodes + parentIndex;
            ref var childInParent = ref (&parentNode->A)[indexInParent];
            newNode->A = childInParent;

            //Insert the new leaf into the second child slot.
            ref var b = ref newNode->B;
            b.Min = newLeafBounds.Min;
            var leafIndex = AddLeaf(newNodeIndex, 1);
            b.Index = Encode(leafIndex);
            b.Max = newLeafBounds.Max;
            b.LeafCount = 1;

            //Update the old leaf node with the new index information.
            var oldLeafIndex = Encode(newNode->A.Index);
            leaves[oldLeafIndex] = new Leaf(newNodeIndex, 0);

            //Update the original node's child pointer and bounding box.
            childInParent.Index = newNodeIndex;
            childInParent.Min = merged.Min;
            childInParent.Max = merged.Max;
            ++childInParent.LeafCount;
            return leafIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe int InsertLeafIntoEmptySlot(ref BoundingBox leafBox, int nodeIndex, int childIndex, Node* node)
        {
            ++node->ChildCount;
            var leafIndex = AddLeaf(nodeIndex, childIndex);
            ref var child = ref (&node->A)[childIndex];
            child.Min = leafBox.Min;
            child.Index = Encode(leafIndex);
            child.Max = leafBox.Max;
            child.LeafCount = 1;
            return leafIndex;
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
            while (true)
            {
                //Which child should the leaf belong to?

                //Give the leaf to whichever node had the least volume change. 
                var node = nodes + nodeIndex;
                var children = &node->A;
                int minimumIndex = 0;
                float minimumChange = float.MaxValue;
                BoundingBox merged = new BoundingBox();
                var bestChoice = BestInsertionChoice.EmptySlot;
                if (node->ChildCount < 2)
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
                        BoundingBox.Merge(ref children[i], ref box, out var mergedCandidate);
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
