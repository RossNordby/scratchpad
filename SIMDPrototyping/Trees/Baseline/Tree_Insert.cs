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


        public unsafe void Insert(T leaf)
        {
            BoundingBox box;
            leaf.GetBoundingBox(out box);

            var node = Levels[0].Nodes;
            BoundingBox rootBounds = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            var boundingBoxes = &node->A;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                BoundingBox.Merge(ref rootBounds, ref boundingBoxes[i], out rootBounds);
            }
            var oldParentCost = ComputeBoundsHeuristic(ref rootBounds);
            BoundingBox.Merge(ref box, ref rootBounds, out rootBounds);
            var parentCost = ComputeBoundsHeuristic(ref rootBounds);
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
                node = level.Nodes + nodeIndex;
                boundingBoxes = &node->A;
                var children = &node->ChildA;
                var leafCounts = &node->LeafCountA;
                int minimumIndex = 0;
                float minimumChange = float.MaxValue;
                BoundingBox merged = new BoundingBox();
                var max = Math.Min(ChildrenCapacity, node->ChildCount + 1);
                for (int i = 0; i < max; ++i)
                {
                    //if (children[i] == -1)
                    //{
                    //    minimumIndex = i;
                    //    merged = box;
                    //    break;
                    //}

                    var oldCost = Math.Max(0, ComputeBoundsHeuristic(ref boundingBoxes[i]));
                    BoundingBox mergedCandidate;
                    BoundingBox.Merge(ref boundingBoxes[i], ref box, out mergedCandidate);
                    var newCost = ComputeBoundsHeuristic(ref mergedCandidate);
                    //var costChange = newCost * (leafCounts[i] + 1) - oldCost * (leafCounts[i]);
                    //var costChange = (newCost - oldCost) / ((leafCounts[i] + 1) / Math.Max(1, leafCounts[i]));
                    //var costChange = (newCost - oldCost) * ((leafCounts[i] + 1) / Math.Max(1, leafCounts[i]));
                    //var costChange = (newCost) * ((leafCounts[i] + 1) / Math.Max(1, leafCounts[i])) - oldCost;
                    //var costChange = newCost * (leafCounts[i] + 1) - oldCost * Math.Max(0, leafCounts[i]);
                    //float costChange;
                    //const float costPerNode = 0;
                    //const float costPerLeaf = 1;
                    //if (children[i] == -1)
                    //{
                    //    //Going this route would simply use the cost of the new addition.
                    //    //TODO: Should this include a 'cost of leaf traversal'?
                    //    costChange = newCost / parentCost;
                    //}
                    //else if (children[i] < -1)
                    //{
                    //    //Merging two leaves. Adds a node, and adds a leaf.
                    //    //costChange = (newCost * (leafCounts[i] + 1) - oldCost * leafCounts[i]) * costPerLeaf / parentCost + costPerNode;
                    //    //costChange = (newCost * 2 * costPerLeaf / parentCost + costPerNode) - oldCost * costPerLeaf / parentCost;
                    //    //costChange = (newCost - oldCost) / parentCost + costPerNode;
                    //    //costChange = newCost / parentCost - oldCost / oldParentCost + costPerNode;
                    //    costChange = newCost * (leafCounts[i] + 1) * costPerLeaf / parentCost - oldCost * leafCounts[i] * costPerLeaf / oldParentCost + costPerNode;
                    //}
                    //else
                    //{
                    //    //Just searching into the next internal node. Adds a leaf to the heuristic cost.
                    //    //costChange = (newCost * (leafCounts[i] + 1) - oldCost * leafCounts[i]) * costPerLeaf / parentCost;
                    //    //costChange = (newCost * (leafCounts[i] + 1) * costPerLeaf / parentCost) - oldCost * leafCounts[i] * costPerLeaf / parentCost;
                    //    //costChange = (newCost - oldCost) / parentCost;
                    //    //costChange = newCost / parentCost - oldCost / oldParentCost;
                    //    costChange = newCost * (leafCounts[i] + 1) * costPerLeaf / parentCost - oldCost * leafCounts[i] * costPerLeaf / oldParentCost;
                    //}
                    var costChange = newCost - oldCost;
                    if (costChange < minimumChange)
                    {
                        minimumChange = costChange;
                        minimumIndex = i;
                        merged = mergedCandidate;
                    }
                }
#if OUTPUT
                Console.WriteLine($"Minimum index: {minimumIndex}, minimum volume increase: {minimum}");
                choices.Add(minimumIndex);
#endif

                var childIndex = children[minimumIndex];

                if (childIndex < -1)
                {
                    //It's a leaf node.
                    //Create a new internal node with the new leaf and the old leaf as children.
                    var nextLevel = levelIndex + 1;
                    //this is the only place where a new level could potentially be created.
                    EnsureLevel(nextLevel);
                    Node newNode;
                    InitializeNode(out newNode);
                    newNode.ChildCount = 2;
                    //The first child of the new node is the old leaf. Insert its bounding box.
                    newNode.A = boundingBoxes[minimumIndex];
                    newNode.ChildA = children[minimumIndex];
                    newNode.LeafCountA = 1;

                    //Insert the new leaf into the second child slot.
                    newNode.B = box;
                    newNode.LeafCountB = 1;
                    var newNodeIndex = Levels[nextLevel].Add(ref newNode);
                    var leafIndex = AddLeaf(leaf, nextLevel, newNodeIndex, 1);
                    Levels[nextLevel].Nodes[newNodeIndex].ChildB = Encode(leafIndex);

                    //Update the old leaf node with the new index information.
                    var oldLeafIndex = Encode(childIndex);
                    leaves[oldLeafIndex].LevelIndex = nextLevel;
                    leaves[oldLeafIndex].NodeIndex = newNodeIndex;
                    leaves[oldLeafIndex].ChildIndex = 0;

                    //Update the original node's child pointer and bounding box.
                    children[minimumIndex] = newNodeIndex;
                    boundingBoxes[minimumIndex] = merged;
                    ++leafCounts[minimumIndex];


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
                    ++node->ChildCount;
                    var leafIndex = AddLeaf(leaf, levelIndex, nodeIndex, minimumIndex);
                    children[minimumIndex] = Encode(leafIndex);
                    boundingBoxes[minimumIndex] = merged;
                    leafCounts[minimumIndex] = 1;


#if OUTPUT
                    Console.WriteLine($"Leaf {leafIndex} inserted in empty slot.");
                    Console.WriteLine($"Choices: {GetChoiceList(choices)}");
#endif
                    break;
                }
                //It's an internal node. Traverse to the next node.
                oldParentCost = ComputeBoundsHeuristic(ref boundingBoxes[minimumIndex]);
                boundingBoxes[minimumIndex] = merged;
                parentCost = ComputeBoundsHeuristic(ref merged);
                nodeIndex = children[minimumIndex];
                ++leafCounts[minimumIndex];
                ++levelIndex;

            }
        }
    }
}
