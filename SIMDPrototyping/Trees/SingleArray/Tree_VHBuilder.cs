//#define NODE8


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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static float ComputeBoundsMetric(ref BoundingBox boundingBox)
        {
            return BoundingBox.ComputeVolume(ref boundingBox);
            //var offset = boundingBox.Max - boundingBox.Min;
            //return (offset.X * offset.Y + offset.Y * offset.Z + offset.Z * offset.X);
        }

        int GetVolumeSplitIndex(int[] leafIds, BoundingBox[] leafBounds, int start, int length)
        {
            CentroidSort(leafIds, leafBounds, start, length);
            //Precompute the bounding boxes to avoid redundant work.
            //TODO: Could avoid reallocating this array over and over again. Doesn't really matter; this should end up being worse than incremental refinement anyway.
            //Build the bMerged lists by iterating backwards through the sorted leaves. Only do length - 1, since 0 < split < length - 1.
            //We want to just index bMerged directly with the candidate splitIndex to get the associated bounding boxes.
            //So, bMerged[0] should contain all bounding boxes from leaves[start + 1] to leaves[start + length - 1].
            //bMerged[bMerged.Length - 1] is just leaves[start + length - 1]'s bounding box.
            var bMerged = new BoundingBox[length - 1];
            var lastIndex = start + length - 1;
            bMerged[bMerged.Length - 1] = leafBounds[lastIndex];
            for (int i = bMerged.Length - 2; i >= 0; --i)
            {
                BoundingBox.Merge(ref leafBounds[start + 1 + i], ref bMerged[i + 1], out bMerged[i]);
            }

            int lowestIndex = -1;
            float lowestCost = float.MaxValue;
            BoundingBox merged = new BoundingBox
            {
                Min = new Vector3(float.MaxValue),
                Max = new Vector3(-float.MaxValue)
            };
            for (int i = 0; i < length - 1; ++i)
            {
                BoundingBox.Merge(ref merged, ref leafBounds[start + i], out merged);
                var candidateCost = i * ComputeBoundsMetric(ref merged) + (length - i) * ComputeBoundsMetric(ref bMerged[i]);
                if (candidateCost < lowestCost)
                {
                    lowestCost = candidateCost;
                    lowestIndex = i;
                }
            }
            return start + lowestIndex + 1;
        }

        unsafe void Split(int[] leafIds, BoundingBox[] leafBounds, int start, int length, int recursionRemaining,
            int* starts, int min, int max)
        {
            if (length <= 1)
            {
                //Splitting is pointless.
                return;
            }
            int splitIndex = GetVolumeSplitIndex(leafIds, leafBounds, start, length);

            int midPoint = (min + max) / 2;
            starts[midPoint] = splitIndex;
            --recursionRemaining;
            if (recursionRemaining > 0)
            {
                Split(leafIds, leafBounds, start, splitIndex - start, recursionRemaining, starts, min, midPoint);
                Split(leafIds, leafBounds, splitIndex, start + length - splitIndex, recursionRemaining, starts, midPoint, max);
            }
        }

        struct Subnode
        {
            public int Start;
            public int Length;
        }
        unsafe void Split(int[] leafIds, BoundingBox[] leafBounds, int start, int length, int recursionRemaining,
         Queue<Subnode> subnodes)
        {
            if (length <= 1)
            {
                //Splitting is pointless.
                return;
            }
            int splitIndex = GetVolumeSplitIndex(leafIds, leafBounds, start, length);

            --recursionRemaining;
            if (recursionRemaining > 0)
            {
                Split(leafIds, leafBounds, start, splitIndex - start, recursionRemaining, subnodes);
                Split(leafIds, leafBounds, splitIndex, start + length - splitIndex, recursionRemaining, subnodes);
            }
        }

        unsafe void VolumeHeuristicAddNode(int parentNodeIndex, int indexInParent, int[] leafIds, BoundingBox[] leafBounds, int start, int length, out BoundingBox mergedBoundingBox, out int nodeIndex)
        {
            bool nodesInvalidated;
            nodeIndex = AllocateNode(out nodesInvalidated);
            Debug.Assert(!nodesInvalidated, "Node capacity should have been ensured prior to the build.");

            var node = nodes + nodeIndex;
            node->Parent = parentNodeIndex;
            node->IndexInParent = indexInParent;

            var boundingBoxes = &node->A;
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;

            if (length <= ChildrenCapacity)
            {
                //Don't need to do any sorting at all. This internal node contains only leaves. 
                mergedBoundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
                for (int i = 0; i < length; ++i)
                {
                    boundingBoxes[i] = leafBounds[i + start];
                    bool leavesInvalidated;
                    MedianSplitAllocateLeafInNode(leafIds[i + start], ref boundingBoxes[i], nodeIndex, out children[i], out leafCounts[i], i, out leavesInvalidated);
                    BoundingBox.Merge(ref boundingBoxes[i], ref mergedBoundingBox, out mergedBoundingBox);
                }
                return;
            }
            //It is an internal node.
            var temp = ChildrenCapacity;
            var recursionDepth = 0;
            while (temp > 1)
            {
                temp >>= 1;
                ++recursionDepth;
            }

            //Compute the split indices.
            Queue<Subnode> childNodes = new Queue<Subnode>();
            childNodes.Enqueue(new Subnode { Start = start, Length = length });

            for (int i = 0; i < recursionDepth; ++i)
            {
                int count = childNodes.Count;
                for (int j = 0; j < count; ++j)
                {
                    var childNode = childNodes.Dequeue();
                    if (childNode.Length > 1)
                    {
                        int splitIndex = GetVolumeSplitIndex(leafIds, leafBounds, childNode.Start, childNode.Length);
                        childNodes.Enqueue(new Subnode { Start = childNode.Start, Length = splitIndex - childNode.Start });
                        childNodes.Enqueue(new Subnode { Start = splitIndex, Length = childNode.Start + childNode.Length - splitIndex });
                    }
                    else
                    {
                        //No point in splitting a node with 1 child.
                        childNodes.Enqueue(childNode);
                    }
                }
            }


            mergedBoundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            int childIndex = 0;
            while (childNodes.Count > 0)
            {
                var childNode = childNodes.Dequeue();

                if (childNode.Length == 1)
                {
                    //Stick the leaf in this slot and continue to the next child.
                    boundingBoxes[childIndex] = leafBounds[childNode.Start];
                    bool leavesInvalidated;
                    MedianSplitAllocateLeafInNode(leafIds[childNode.Start], ref boundingBoxes[childIndex], nodeIndex, out children[childIndex], out leafCounts[childIndex], childIndex, out leavesInvalidated);
                }
                else
                {
                    //Multiple children fit this slot. Create another internal node.
                    VolumeHeuristicAddNode(nodeIndex, childIndex, leafIds, leafBounds, childNode.Start, childNode.Length, out boundingBoxes[childIndex], out children[childIndex]);
                    ++node->ChildCount;
                    leafCounts[childIndex] = childNode.Length;
                }
                BoundingBox.Merge(ref boundingBoxes[childIndex], ref mergedBoundingBox, out mergedBoundingBox);
                ++childIndex;
            }

        }


        public unsafe void BuildVolumeHeuristic(int[] leafIds, BoundingBox[] leafBounds, int start = 0, int length = -1)
        {
            if (leafIds.Length != leafBounds.Length)
                throw new ArgumentException("leafIds and leafBounds lengths must be equal.");
            if (start + length > leafIds.Length)
                throw new ArgumentException("Start + length must be smaller than the leaves array length.");
            if (start < 0)
                throw new ArgumentException("Start must be nonnegative.");
            if (length == 0)
                throw new ArgumentException("Length must be positive.");
            if (length < 0)
                length = leafIds.Length;
            if (nodes[0].ChildCount != 0)
                throw new InvalidOperationException("Cannot build a tree that already contains nodes.");
            //The tree is built with an empty node at the root to make insertion work more easily.
            //As long as that is the case (and as long as this is not a constructor),
            //we must clear it out.
            nodeCount = 0;

            //Guarantee that no resizes will occur during the build.
            if (LeafCapacity < leafBounds.Length)
            {
                LeafCapacity = leafBounds.Length;
            }
            var preallocatedNodeCount = leafBounds.Length * 2 - 1;
            if (NodeCapacity < preallocatedNodeCount)
            {
                NodeCapacity = preallocatedNodeCount;
            }

            int nodeIndex;
            BoundingBox boundingBox;
            VolumeHeuristicAddNode(-1, -1, leafIds, leafBounds, start, length, out boundingBox, out nodeIndex);



        }
    }
}
