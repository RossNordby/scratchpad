#define NODE4


using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
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
        float ComputeVolumeHeuristic(int leafCount, ref BoundingBox boundingBox)
        {
            return leafCount * BoundingBox.ComputeVolume(ref boundingBox);
            //var offset = boundingBox.Max - boundingBox.Min;
            //return leafCount * (offset.X * offset.Y + offset.Y * offset.Z + offset.Z * offset.X);
        }

        int GetVolumeSplitIndex(T[] leaves, int start, int length)
        {
            CentroidSort(leaves, start, length);
            //Precompute the bounding boxes to avoid redundant work.
            //TODO: Could avoid reallocating this array over and over again. Doesn't really matter; this should end up being worse than incremental refinement anyway.
            //Build the bMerged lists by iterating backwards through the sorted leaves. Only do length - 1, since 0 < split < length - 1.
            //We want to just index bMerged directly with the candidate splitIndex to get the associated bounding boxes.
            //So, bMerged[0] should contain all bounding boxes from leaves[start + 1] to leaves[start + length - 1].
            //bMerged[bMerged.Length - 1] is just leaves[start + length - 1]'s bounding box.
            var bMerged = new BoundingBox[length - 1];
            var lastIndex = start + length - 1;
            leaves[lastIndex].GetBoundingBox(out bMerged[bMerged.Length - 1]);
            for (int i = bMerged.Length - 2; i >= 0; --i)
            {
                BoundingBox candidate;
                leaves[start + 1 + i].GetBoundingBox(out candidate);
                BoundingBox.Merge(ref candidate, ref bMerged[i + 1], out bMerged[i]);
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
                BoundingBox leafBoundingBox;
                leaves[start + i].GetBoundingBox(out leafBoundingBox);
                BoundingBox.Merge(ref merged, ref leafBoundingBox, out merged);
                var candidateCost = ComputeVolumeHeuristic(i, ref merged) + ComputeVolumeHeuristic(length - i, ref bMerged[i]);
                if (candidateCost < lowestCost)
                {
                    lowestCost = candidateCost;
                    lowestIndex = i;
                }
            }
            return start + lowestIndex + 1;
        }

        unsafe void Split(T[] leaves, int start, int length, int recursionRemaining,
            int* starts, int min, int max)
        {
            if (length <= 1)
            {
                //Splitting is pointless.
                return;
            }
            int splitIndex = GetVolumeSplitIndex(leaves, start, length);

            int midPoint = (min + max) / 2;
            starts[midPoint] = splitIndex;
            --recursionRemaining;
            if (recursionRemaining > 0)
            {
                Split(leaves, start, splitIndex - start, recursionRemaining, starts, min, midPoint);
                Split(leaves, splitIndex, start + length - splitIndex, recursionRemaining, starts, midPoint, max);
            }
        }

        struct Subnode
        {
            public int Start;
            public int Length;
        }
        unsafe void Split(T[] leaves, int start, int length, int recursionRemaining,
         Queue<Subnode> subnodes)
        {
            if (length <= 1)
            {
                //Splitting is pointless.
                return;
            }
            int splitIndex = GetVolumeSplitIndex(leaves, start, length);

            --recursionRemaining;
            if (recursionRemaining > 0)
            {
                Split(leaves, start, splitIndex - start, recursionRemaining, subnodes);
                Split(leaves, splitIndex, start + length - splitIndex, recursionRemaining, subnodes);
            }
        }


        unsafe void VolumeHeuristicAddNode(int level, T[] leaves, int start, int length, out BoundingBox mergedBoundingBox, out int nodeIndex)
        {
            if (level > 150)
            {
                Console.WriteLine("Something is wrong.:)");
            }
            EnsureLevel(level);
            Node node;
            InitializeNode(out node);
            nodeIndex = Levels[level].Add(ref node); //This is a kinda stupid design! Inserting an empty node so we can go back and fill it later!
            var boundingBoxes = &Levels[level].Nodes[nodeIndex].A;
            var children = &Levels[level].Nodes[nodeIndex].ChildA;

            if (length <= ChildrenCapacity)
            {
                //Don't need to do any sorting at all. This internal node contains only leaves. 
                mergedBoundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
                for (int i = 0; i < length; ++i)
                {
                    MedianSplitAllocateLeafInNode(leaves[i + start], level, nodeIndex, out boundingBoxes[i], out children[i], i);
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
                        int splitIndex = GetVolumeSplitIndex(leaves, childNode.Start, childNode.Length);
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
                    MedianSplitAllocateLeafInNode(leaves[childNode.Start], level, nodeIndex, out boundingBoxes[childIndex], out children[childIndex], childIndex);
                }
                else
                {
                    //Multiple children fit this slot. Create another internal node.
                    VolumeHeuristicAddNode(level + 1, leaves, childNode.Start, childNode.Length, out boundingBoxes[childIndex], out children[childIndex]);
                    ++Levels[level].Nodes[nodeIndex].ChildCount;
                }
                BoundingBox.Merge(ref boundingBoxes[childIndex], ref mergedBoundingBox, out mergedBoundingBox);
                ++childIndex;
            }

        }

        public unsafe void BuildVolumeHeuristic(T[] leaves, int start = 0, int length = -1)
        {
            if (start + length > leaves.Length)
                throw new ArgumentException("Start + length must be smaller than the leaves array length.");
            if (start < 0)
                throw new ArgumentException("Start must be nonnegative.");
            if (length == 0)
                throw new ArgumentException("Length must be positive.");
            if (length < 0)
                length = leaves.Length;
            if (Levels[0].Nodes[0].ChildCount != 0)
                throw new InvalidOperationException("Cannot build a tree that already contains nodes.");
            //The tree is built with an empty node at the root to make insertion work more easily.
            //As long as that is the case (and as long as this is not a constructor),
            //we must clear it out.
            Levels[0].Count = 0;

            int nodeIndex;
            BoundingBox boundingBox;
            VolumeHeuristicAddNode(0, leaves, start, length, out boundingBox, out nodeIndex);



        }
    }
}
