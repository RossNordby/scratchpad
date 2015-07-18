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

#if NODE32
using Node = SIMDPrototyping.Trees.Baseline.Node32;
#elif NODE16
using Node = SIMDPrototyping.Trees.Baseline.Node16;
#elif NODE8
using Node = SIMDPrototyping.Trees.Baseline.Node8;
#elif NODE4
using Node = SIMDPrototyping.Trees.Baseline.Node4;
#elif NODE2
using Node = SIMDPrototyping.Trees.Baseline.Node2;
#endif

namespace SIMDPrototyping.Trees.Baseline
{
    partial class Tree<T>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void MergeLeafNodes(T newLeaf, ref BoundingBox newLeafBounds, int parentLevelIndex, int parentIndex, int indexInParent,
            ref int oldLeafChildSlot, ref BoundingBox oldLeafBoundsSlot, ref int oldLeafLeafCountsSlot, ref BoundingBox merged)
        {
            //It's a leaf node.
            //Create a new internal node with the new leaf and the old leaf as children.
            var nextLevel = parentLevelIndex + 1;
            //this is the only place where a new level could potentially be created.
            EnsureLevel(nextLevel);
            Node newNode;
            InitializeNode(out newNode);
            newNode.ChildCount = 2;
            newNode.Parent = parentIndex;
            newNode.IndexInParent = indexInParent;
            //The first child of the new node is the old leaf. Insert its bounding box.
            newNode.A = oldLeafBoundsSlot;
            newNode.ChildA = oldLeafChildSlot;
            newNode.LeafCountA = 1;

            //Insert the new leaf into the second child slot.
            newNode.B = newLeafBounds;
            newNode.LeafCountB = 1;
            var newNodeIndex = Levels[nextLevel].Add(ref newNode);
            var leafIndex = AddLeaf(newLeaf, nextLevel, newNodeIndex, 1);
            Levels[nextLevel].Nodes[newNodeIndex].ChildB = Encode(leafIndex);

            //Update the old leaf node with the new index information.
            var oldLeafIndex = Encode(oldLeafChildSlot);
            leaves[oldLeafIndex].LevelIndex = nextLevel;
            leaves[oldLeafIndex].NodeIndex = newNodeIndex;
            leaves[oldLeafIndex].ChildIndex = 0;

            //Update the original node's child pointer and bounding box.
            oldLeafChildSlot = newNodeIndex;
            oldLeafBoundsSlot = merged;
            ++oldLeafLeafCountsSlot;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void InsertLeafIntoEmptySlot(T leaf, ref BoundingBox leafBox, int levelIndex, int nodeIndex, int childIndex, Node* node)
        {
            ++node->ChildCount;
            var leafIndex = AddLeaf(leaf, levelIndex, nodeIndex, childIndex);
            (&node->ChildA)[childIndex] = Encode(leafIndex);
            (&node->A)[childIndex] = leafBox;
            (&node->LeafCountA)[childIndex] = 1;
        }

        public unsafe void Insert(T leaf)
        {
            BoundingBox box;
            leaf.GetBoundingBox(out box);


            int levelIndex = 0;
            int nodeIndex = 0;
#if OUTPUT
            List<int> choices = new List<int>();
#endif
            while (true)
            {
                var level = Levels[levelIndex];
                //Which child should the leaf belong to?

                //Give the leaf to whichever node had the least volume change.          
                var node = Levels[0].Nodes;
                var boundingBoxes = &node->A;
                node = level.Nodes + nodeIndex;
                boundingBoxes = &node->A;
                var children = &node->ChildA;
                var leafCounts = &node->LeafCountA;
                int minimumIndex = 0;
                float minimumChange = float.MaxValue;
                BoundingBox merged = new BoundingBox();
                if (node->ChildCount < ChildrenCapacity)
                {
                    //The best slot will, at best, be tied with inserting it in a leaf node because the change in heuristic cost
                    //for filling an empty slot is zero.
                    minimumIndex = node->ChildCount;
                    merged = box;
                    minimumChange = 0;
                }
                else
                {
                    for (int i = 0; i < node->ChildCount; ++i)
                    {
                        BoundingBox mergedCandidate;
                        BoundingBox.Merge(ref boundingBoxes[i], ref box, out mergedCandidate);
                        var newCost = ComputeBoundsHeuristic(ref mergedCandidate);
                        //Since we already checked for an empty slot, the two remaining possibilities are merging with an existing leaf node
                        //and continuing down another internal node.
                        //Going into another internal node only increases the relevant cost (that of internal nodes) by the change in merged volume.
                        //Merging into a leaf node generates a whole new internal node, so it tends to be more expensive.
                        float costChange = children[i] >= 0 ? newCost - ComputeBoundsHeuristic(ref boundingBoxes[i]) : newCost;


                        if (costChange < minimumChange)
                        {
                            minimumChange = costChange;
                            minimumIndex = i;
                            merged = mergedCandidate;
                        }

                    }

                }

#if OUTPUT
                Console.WriteLine($"Minimum index: {minimumIndex}, minimum volume increase: {minimum}");
                choices.Add(minimumIndex);
#endif

                var childIndex = children[minimumIndex];

                if (childIndex < -1)
                {
                    MergeLeafNodes(leaf, ref box, levelIndex, nodeIndex, minimumIndex, ref children[minimumIndex], ref boundingBoxes[minimumIndex], ref leafCounts[minimumIndex], ref merged);

#if OUTPUT
                    Console.WriteLine($"Leaf {leafIndex} merged with existing leaf.");// New Node Children: {newNode.Children}, Old Node children: {level.Nodes[nodeIndex].Children}");
                    Console.WriteLine($"Choices: {GetChoiceList(choices)}");
#endif

                    break;
                }
                if (childIndex == -1)
                {
                    //There is no child at all.
                    //Put the new leaf here.
                    InsertLeafIntoEmptySlot(leaf, ref box, levelIndex, nodeIndex, minimumIndex, node);


#if OUTPUT
                    Console.WriteLine($"Leaf {leafIndex} inserted in empty slot.");
                    Console.WriteLine($"Choices: {GetChoiceList(choices)}");
#endif
                    break;
                }
                //It's an internal node. Traverse to the next node.
                boundingBoxes[minimumIndex] = merged;
                nodeIndex = children[minimumIndex];
                ++leafCounts[minimumIndex];
                ++levelIndex;

            }
        }
    }
}
