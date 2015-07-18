using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace SIMDPrototyping.Trees.Baseline
{
    partial class Tree<T>
    {
        string GetChoiceList(List<int> choices)
        {
            StringBuilder builder = new StringBuilder(choices.Count * 2);
            for (int i = 0; i < choices.Count; ++i)
            {
                builder.Append(choices[i]);
                if (i != choices.Count - 1)
                    builder.Append(", ");
            }
            return builder.ToString();
        }



        public unsafe float MeasureCostHeuristic()
        {
            var rootNode = Levels[0].Nodes;
            var rootBounds = &rootNode->A;

            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < rootNode->ChildCount; ++i)
            {
                BoundingBox.Merge(ref rootBounds[i], ref merged, out merged);
            }
            float rootHeuristic = ComputeBoundsHeuristic(ref merged);

            const float leafCost = 1;
            const float internalNodeCost = 1;

            float totalCost = 0;
            for (int levelIndex = 0; levelIndex <= maximumDepth; ++levelIndex)
            {
                var level = Levels[levelIndex];
                for (int nodeIndex = 0; nodeIndex < level.Count; ++nodeIndex)
                {
                    var node = level.Nodes + nodeIndex;
                    var children = &node->ChildA;
                    var bounds = &node->A;
                    for (int childIndex = 0; childIndex < level.Nodes[nodeIndex].ChildCount; ++childIndex)
                    {
                        if (children[childIndex] >= 0)
                        {
                            //Internal node.
                            totalCost += internalNodeCost * ComputeBoundsHeuristic(ref bounds[childIndex]);
                        }
                        else
                        {
                            //Leaf node.
                            totalCost += leafCost * ComputeBoundsHeuristic(ref bounds[childIndex]);
                        }

                    }
                }
            }
            return totalCost / rootHeuristic;

        }

        unsafe void Validate(int levelIndex, int nodeIndex, int expectedParentIndex, int expectedIndexInParent, ref BoundingBox expectedBoundingBox, out int foundLeafCount)
        {
            var node = Levels[levelIndex].Nodes + nodeIndex;
            if (node->Parent != expectedParentIndex)
                throw new Exception($"Bad parent index on node ({levelIndex}, {nodeIndex}).");
            if (node->IndexInParent != expectedIndexInParent)
                throw new Exception($"Bad index in parent on node ({levelIndex}, {nodeIndex}).");
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            var bounds = &node->A;
            var nextLevel = levelIndex + 1;
            foundLeafCount = 0;
            if (node->ChildCount < 2 || node->ChildCount > ChildrenCapacity)
            {
                throw new Exception($"Internal node with {node->ChildCount} children.");
            }
            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            for (int i = 0; i < node->ChildCount; ++i)
            {
                BoundingBox.Merge(ref merged, ref bounds[i], out merged);
                if (children[i] >= 0)
                {
                    int childFoundLeafCount;
                    Validate(nextLevel, children[i], nodeIndex, i, ref bounds[i], out childFoundLeafCount);
                    if (childFoundLeafCount != leafCounts[i])
                        throw new Exception($"Bad leaf count for child {i} of node ({levelIndex}, {nodeIndex}).");
                    foundLeafCount += childFoundLeafCount;
                }
                else if (children[i] < -1)
                {
                    ++foundLeafCount;
                }
                else if (children[i] == -1)
                {
                    throw new Exception($"Empty child at index {i} within count on node ({levelIndex}, {nodeIndex}).");
                }
            }

            if (expectedParentIndex >= 0 && (merged.Min != expectedBoundingBox.Min || merged.Max != expectedBoundingBox.Max))
            {
                throw new Exception($"Bounds {merged.ToString()}, expected {expectedBoundingBox.ToString()}.");
            }
        }



        public void Validate()
        {
            int foundLeafCount;
            var standInBounds = new BoundingBox();
            Validate(0, 0, -1, -1, ref standInBounds, out foundLeafCount);
            if (foundLeafCount != LeafCount)
                throw new Exception($"{foundLeafCount} leaves found in tree, expected {leafCount}.");
        }

    }
}
