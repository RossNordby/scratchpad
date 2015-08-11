using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;



namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
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


        //TODO: Note that this heuristic does not fully capture the cost of a node.
        //It assumes that traversing a node with 2 children is about the same as traversing a node with 8 children.
        //While this may be closer to true that it appears at first glance due to the very high cost of cache misses versus trivial ALU work,
        //it's probably not *identical*.
        //The builders also use this approximation.
        public unsafe float MeasureCostMetric()
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            var rootNode = nodes;
            var rootBounds = &rootNode->A;

            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < rootNode->ChildCount; ++i)
            {
                BoundingBox.Merge(ref rootBounds[i], ref merged, out merged);
            }
            float rootHeuristic = ComputeBoundsMetric(ref merged);

            const float leafCost = 1;
            const float internalNodeCost = 1;

            float totalCost = 0;
            for (int i = 0; i < nodeCount; ++i)
            {
                var node = nodes + i;
                var children = &node->ChildA;
                var bounds = &node->A;
                for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
                {
                    if (children[childIndex] >= 0)
                    {
                        //Internal node.
                        totalCost += internalNodeCost * ComputeBoundsMetric(ref bounds[childIndex]);
                    }
                    else
                    {
                        //Leaf node.
                        totalCost += leafCost * ComputeBoundsMetric(ref bounds[childIndex]);
                    }

                }
            }
            return totalCost / rootHeuristic;

        }

        unsafe void Validate(int nodeIndex, int expectedParentIndex, int expectedIndexInParent, ref BoundingBox expectedBoundingBox, out int foundLeafCount)
        {
            var node = nodes + nodeIndex;
            if (node->Parent != expectedParentIndex)
                throw new Exception($"Bad parent index on node {nodeIndex}");
            if (node->IndexInParent != expectedIndexInParent)
                throw new Exception($"Bad index in parent on node {nodeIndex}");
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            var bounds = &node->A;
            foundLeafCount = 0;
            if ((expectedParentIndex >= 0 && node->ChildCount < 2) || node->ChildCount < 0 || node->ChildCount > ChildrenCapacity)
            {
                throw new Exception($"Internal node with {node->ChildCount} children.");
            }
            var badMinValue = new Vector3(float.MaxValue);
            var badMaxValue = new Vector3(float.MinValue);
            BoundingBox merged = new BoundingBox { Min = badMinValue, Max = badMaxValue };
            for (int i = 0; i < node->ChildCount; ++i)
            {
                BoundingBox.Merge(ref merged, ref bounds[i], out merged);
                if (bounds[i].Min == badMinValue || bounds[i].Max == badMaxValue)
                    throw new Exception($"Node {nodeIndex} child {i} has a bad bounding box.");
                if (children[i] >= 0)
                {
                    int childFoundLeafCount;
                    if (children[i] < 0 || children[i] >= nodeCount)
                        throw new Exception($"Implied existence of node {children[i]} is outside of count {nodeCount}.");
                    Validate(children[i], nodeIndex, i, ref bounds[i], out childFoundLeafCount);
                    if (childFoundLeafCount != leafCounts[i])
                        throw new Exception($"Bad leaf count for child {i} of node {nodeIndex}.");
                    foundLeafCount += childFoundLeafCount;
                }
                else
                {
                    ++foundLeafCount;
                    if (leafCounts[i] != 1)
                    {
                        throw new Exception($"Bad leaf count on {nodeIndex} child {i}, it's a leaf but leafCount is {leafCounts[i]}.");
                    }
                }
            }

            if (expectedParentIndex >= 0 && (merged.Min != expectedBoundingBox.Min || merged.Max != expectedBoundingBox.Max))
            {
                throw new Exception($"Bounds {merged.ToString()}, expected {expectedBoundingBox.ToString()}.");
            }
        }

        unsafe void ValidateLeafNodeIndices()
        {
            for (int i = 0; i < leafCount; ++i)
            {
                if (leaves[i].NodeIndex < 0)
                {
                    throw new Exception($"Leaf {i} has negative node index: {leaves[i].NodeIndex}.");
                }
                if (leaves[i].NodeIndex >= nodeCount)
                {
                    throw new Exception($"Leaf {i} points to a node outside the node set, {leaves[i].NodeIndex} >= {nodeCount}.");
                }
            }
        }

        unsafe void ValidateLeaves()
        {
            ValidateLeafNodeIndices();

            for (int i = 0; i < leafCount; ++i)
            {
                if (Encode((&nodes[leaves[i].NodeIndex].ChildA)[leaves[i].ChildIndex]) != i)
                {
                    throw new Exception($"Leaf {i} data does not agree with node about parenthood.");
                }
            }
        }

        public unsafe void Validate()
        {
            if (nodeCount < 0)
            {
                throw new Exception($"Invalid negative node count of {nodeCount}");
            }
            else if (nodeCount > Nodes.Length)
            {
                throw new Exception($"Invalid node count of {nodeCount}, larger than nodes array length {Nodes.Length}.");
            }
            if (LeafCount > 0 && (nodes[0].IndexInParent != -1 || nodes[0].IndexInParent != -1))
            {
                throw new Exception($"Invalid parent pointers on root.");
            }

            int foundLeafCount;
            var standInBounds = new BoundingBox();

            Validate(0, -1, -1, ref standInBounds, out foundLeafCount);
            if (foundLeafCount != leafCount)
                throw new Exception($"{foundLeafCount} leaves found in tree, expected {leafCount}.");

            ValidateLeaves();

        }


        unsafe void MeasureNodeOccupancy(Node* node, out int nodeCount, out int childCount)
        {
            var children = &node->ChildA;
            nodeCount = 1;
            childCount = node->ChildCount;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    int childNodeCount, childChildCount;
                    MeasureNodeOccupancy(nodes + children[i], out childNodeCount, out childChildCount);
                    nodeCount += childNodeCount;
                    childCount += childChildCount;
                }
            }
        }

        public unsafe void MeasureNodeOccupancy(out int nodeCount, out int childCount)
        {
            MeasureNodeOccupancy(nodes, out nodeCount, out childCount);


        }

        unsafe int ComputeMaximumDepth(Node* node, int currentDepth)
        {
            var children = &node->ChildA;
            int maximum = currentDepth;
            int nextDepth = currentDepth + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    var candidate = ComputeMaximumDepth(nodes + children[i], nextDepth);
                    if (candidate > maximum)
                        maximum = candidate;
                }
            }
            return maximum;
        }

        public unsafe int ComputeMaximumDepth()
        {
            return ComputeMaximumDepth(nodes, 0);
        }

        unsafe void MeasureCacheQuality(int nodeIndex, out int foundNodes, out float nodeScore, out int scoreableNodeCount)
        {
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            nodeScore = 0;
            scoreableNodeCount = 0;
            foundNodes = 0;
            int correctlyPositionedImmediateChildren = 0;
            int immediateInternalChildren = 0;
            int expectedChildIndex = nodeIndex + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    ++immediateInternalChildren;
                    if (children[i] == expectedChildIndex)
                    {
                        ++correctlyPositionedImmediateChildren;
                    }
                    int childFoundNodes;
                    float childNodeScore;
                    int childScoreableNodes;
                    MeasureCacheQuality(children[i], out childFoundNodes, out childNodeScore, out childScoreableNodes);
                    foundNodes += childFoundNodes;
                    expectedChildIndex += childFoundNodes;
                    nodeScore += childNodeScore;
                    scoreableNodeCount += childScoreableNodes;
                }

            }

            ++foundNodes;
            //Include this node.
            if (immediateInternalChildren > 0)
            {
                nodeScore += correctlyPositionedImmediateChildren / (float)immediateInternalChildren;
                ++scoreableNodeCount;
            }
        }
        public unsafe float MeasureCacheQuality()
        {
            float nodeScore;
            int foundNodes, scoreableNodeCount;
            MeasureCacheQuality(0, out foundNodes, out nodeScore, out scoreableNodeCount);
            return nodeScore / (float)scoreableNodeCount;

        }

    }
}
