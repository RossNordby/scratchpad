﻿using BEPUutilities2;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;



namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        //TODO: Note that this heuristic does not fully capture the cost of a node.
        //It assumes that traversing a node with 2 children is about the same as traversing a node with 8 children.
        //While this may be closer to true that it appears at first glance due to the very high cost of cache misses versus trivial ALU work,
        //it's probably not *identical*.
        //The builders also use this approximation.
        public unsafe float MeasureCostMetric()
        {
            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            var rootNode = nodes;
            var rootChildren = &rootNode->A;

            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < leafCount; ++i)
            {
                ref var child = ref rootChildren[i];
                BoundingBox.CreateMerged(ref child.Min, ref child.Max, ref merged.Min, ref merged.Max, out merged.Min, out merged.Max);
            }
            float rootMetric = ComputeBoundsMetric(ref merged);

            const float leafCost = 1;
            const float internalNodeCost = 1;

            if (leafCount > 2)
            {
                float totalCost = 0;
                for (int i = 0; i < nodeCount; ++i)
                {
                    var node = nodes + i;
                    var children = &node->A;
                    var bounds = &node->A;
                    for (int childIndex = 0; childIndex < 2; ++childIndex)
                    {
                        ref var child = ref children[childIndex];
                        if (child.Index >= 0)
                        {
                            //Internal node.
                            totalCost += internalNodeCost * ComputeBoundsMetric(ref child.Min, ref child.Max);
                        }
                        else
                        {
                            //Leaf node.
                            totalCost += leafCost * ComputeBoundsMetric(ref child.Min, ref child.Max);
                        }

                    }
                }
                return totalCost / rootMetric;
            }
            return 0;

        }
        
        unsafe void Validate(int nodeIndex, int expectedParentIndex, int expectedIndexInParent, ref Vector3 expectedMin, ref Vector3 expectedMax, out int foundLeafCount)
        {
            var node = nodes + nodeIndex;
            if (node->Parent != expectedParentIndex)
                throw new Exception($"Bad parent index on node {nodeIndex}");
            if (node->IndexInParent != expectedIndexInParent)
                throw new Exception($"Bad index in parent on node {nodeIndex}");
            if (node->RefineFlag != 0)
                throw new Exception($"Nonzero refine flag on node {nodeIndex}");
            var children = &node->A;
            foundLeafCount = 0;
            var badMinValue = new Vector3(float.MaxValue);
            var badMaxValue = new Vector3(float.MinValue);
            BoundingBox merged = new BoundingBox { Min = badMinValue, Max = badMaxValue };
            for (int i = 0; i < node->ChildCount; ++i)
            {
                ref var child = ref children[i];
                if (child.Min == badMinValue || child.Max == badMaxValue)
                    throw new Exception($"Node {nodeIndex} child {i} has a bad bounding box.");
                BoundingBox.CreateMerged(ref merged.Min, ref merged.Max, ref child.Min, ref child.Max, out merged.Min, out merged.Max);
                if (child.Index >= 0)
                {
                    if (child.Index >= nodeCount)
                        throw new Exception($"Implied existence of node {children[i]} is outside of count {nodeCount}.");
                    Validate(child.Index, nodeIndex, i, ref child.Min, ref child.Max, out int childFoundLeafCount);
                    if (childFoundLeafCount != child.LeafCount)
                        throw new Exception($"Bad leaf count for child {i} of node {nodeIndex}.");
                    foundLeafCount += childFoundLeafCount;
                }
                else
                {
                    ++foundLeafCount;
                    if (child.LeafCount != 1)
                    {
                        throw new Exception($"Bad leaf count on {nodeIndex} child {i}, it's a leaf but leafCount is {child.LeafCount}.");
                    }
                }
            }

            if (expectedParentIndex >= 0 && //Not a root node,
                (merged.Min != expectedMin || merged.Max != expectedMax))
            {
                throw new Exception($"Bounds {merged.ToString()}, expected ({expectedMin}, {expectedMax}).");
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
                if (Encode((&nodes[leaves[i].NodeIndex].A)[leaves[i].ChildIndex].Index) != i)
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
            if((nodeCount != 1 && leafCount < 2) || (nodeCount != LeafCount - 1 && leafCount >= 2))
            {
                throw new Exception($"Invalid node count versus leaf count.");
            }

            var standInBounds = new BoundingBox();

            Validate(0, -1, -1, ref standInBounds.Min, ref standInBounds.Max, out int foundLeafCount);
            if (foundLeafCount != leafCount)
                throw new Exception($"{foundLeafCount} leaves found in tree, expected {leafCount}.");

            ValidateLeaves();

        }


        unsafe void MeasureNodeOccupancy(Node* node, out int nodeCount, out int childCount)
        {
            var children = &node->A;
            nodeCount = 1;
            childCount = node->ChildCount;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    MeasureNodeOccupancy(nodes + child.Index, out int childNodeCount, out int childChildCount);
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
            var children = &node->A;
            int maximum = currentDepth;
            int nextDepth = currentDepth + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    var candidate = ComputeMaximumDepth(nodes + child.Index, nextDepth);
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

        unsafe void MeasureCacheQuality(int nodeIndex, out int foundNodes, out float nodeScore, out int scorableNodeCount)
        {
            var node = nodes + nodeIndex;
            var children = &node->A;
            nodeScore = 0;
            scorableNodeCount = 0;
            foundNodes = 0;
            int correctlyPositionedImmediateChildren = 0;
            int immediateInternalChildren = 0;
            int expectedChildIndex = nodeIndex + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    ++immediateInternalChildren;
                    if (child.Index == expectedChildIndex)
                    {
                        ++correctlyPositionedImmediateChildren;
                    }
                    MeasureCacheQuality(child.Index, out int childFoundNodes, out float childNodeScore, out int childScorableNodes);
                    foundNodes += childFoundNodes;
                    expectedChildIndex += childFoundNodes;
                    nodeScore += childNodeScore;
                    scorableNodeCount += childScorableNodes;
                }

            }

            ++foundNodes;
            //Include this node.
            if (immediateInternalChildren > 0)
            {
                nodeScore += correctlyPositionedImmediateChildren / (float)immediateInternalChildren;
                ++scorableNodeCount;
            }
        }
        public unsafe float MeasureCacheQuality()
        {
            MeasureCacheQuality(0, out int foundNodes, out float nodeScore, out int scorableNodeCount);
            return scorableNodeCount > 0 ? nodeScore / scorableNodeCount : 1;

        }

        public unsafe float MeasureCacheQuality(int nodeIndex)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeCount)
                throw new ArgumentException("Measurement target index must be nonnegative and less than node count.");
            MeasureCacheQuality(nodeIndex, out int foundNodes, out float nodeScore, out int scorableNodeCount);
            return scorableNodeCount > 0 ? nodeScore / scorableNodeCount : 1;
        }

    }
}
