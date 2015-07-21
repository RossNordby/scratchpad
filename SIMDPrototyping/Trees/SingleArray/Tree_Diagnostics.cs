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



        public unsafe float MeasureCostHeuristic()
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            var rootNode = Nodes;
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
            for (int i = 0; i < NodeCount; ++i)
            {
                var node = Nodes + i;
                var children = &node->ChildA;
                var bounds = &node->A;
                for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
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
            return totalCost / rootHeuristic;

        }

        unsafe void Validate(int nodeIndex, int expectedParentIndex, int expectedIndexInParent, ref BoundingBox expectedBoundingBox, out int foundLeafCount)
        {
            var node = Nodes + nodeIndex;
            if (node->Parent != expectedParentIndex)
                throw new Exception($"Bad parent index on node {nodeIndex}");
            if (node->IndexInParent != expectedIndexInParent)
                throw new Exception($"Bad index in parent on node {nodeIndex}");
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            var bounds = &node->A;
            foundLeafCount = 0;
            if (expectedParentIndex >= 0 && (node->ChildCount < 2 || node->ChildCount > ChildrenCapacity))
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
                    if (children[i] < 0 || children[i] >= NodeCount)
                        throw new Exception($"Implied existence of node {children[i]} is outside of count {NodeCount}.");
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


        unsafe void ValidateLeaves()
        {

            for (int i = 0; i < leafCount; ++i)
            {
                if (Leaves[i].NodeIndex < 0)
                {
                    throw new Exception($"Leaf {i} has negative node index: {Leaves[i].NodeIndex}.");
                }
                if (Leaves[i].NodeIndex >= NodeCount)
                {
                    throw new Exception($"Leaf {i} points to a node outside the node set, {Leaves[i].NodeIndex} >= {NodeCount}.");
                }
                if (Encode((&Nodes[Leaves[i].NodeIndex].ChildA)[Leaves[i].ChildIndex]) != i)
                {
                    throw new Exception($"Leaf {i} data does not agree with node about parenthood.");
                }
            }
        }

        public unsafe void Validate()
        {
            if (NodeCount < 0)
            {
                throw new Exception($"Invalid negative node count of {NodeCount}");
            }
            else if (NodeCount > NodesArray.Length)
            {
                throw new Exception($"Invalid node count of {NodeCount}, larger than nodes array length {NodesArray.Length}.");
            }


            ValidateLeaves();

            int foundLeafCount;
            var standInBounds = new BoundingBox();

            Validate(0, -1, -1, ref standInBounds, out foundLeafCount);
            if (foundLeafCount != LeafCount)
                throw new Exception($"{foundLeafCount} leaves found in tree, expected {leafCount}.");


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
                    MeasureNodeOccupancy(Nodes + children[i], out childNodeCount, out childChildCount);
                    nodeCount += childNodeCount;
                    childCount += childChildCount;
                }
            }
        }

        public unsafe void MeasureNodeOccupancy(out int nodeCount, out int childCount)
        {
            MeasureNodeOccupancy(Nodes, out nodeCount, out childCount);


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
                    var candidate = ComputeMaximumDepth(Nodes + children[i], nextDepth);
                    if (candidate > maximum)
                        maximum = candidate;
                }
            }
            return maximum;
        }

        public unsafe int ComputeMaximumDepth()
        {
            return ComputeMaximumDepth(Nodes, 0);
        }

    }
}
