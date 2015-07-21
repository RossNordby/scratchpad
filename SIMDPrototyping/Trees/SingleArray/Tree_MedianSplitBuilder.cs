using System;
using System.Collections.Generic;
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
        static void ComputeCentroid(ref BoundingBox boundingBox, out Vector3 centroid)
        {
            centroid = boundingBox.Min + boundingBox.Max;
        }
        //TODO: This sort could be a LOT faster if we could use pointers in the sort. May be able to make use of some properties of the repeated sorting, too... 
        //custom sort likely a good idea if this type of builder still exists in the final version.
        unsafe class AxisComparerX : IComparer<BoundingBox>
        {
            public unsafe int Compare(BoundingBox a, BoundingBox b)
            {
                return (a.Min.X + a.Max.X).CompareTo(b.Min.X + b.Max.X);

            }
        }
        unsafe class AxisComparerY : IComparer<BoundingBox>
        {
            public unsafe int Compare(BoundingBox a, BoundingBox b)
            {
                return (a.Min.Y + a.Max.Y).CompareTo(b.Min.Y + b.Max.Y);

            }
        }
        unsafe class AxisComparerZ : IComparer<BoundingBox>
        {
            public unsafe int Compare(BoundingBox a, BoundingBox b)
            {
                return (a.Min.Z + a.Max.Z).CompareTo(b.Min.Z + b.Max.Z);

            }
        }
        static AxisComparerX xComparer = new AxisComparerX();
        static AxisComparerY yComparer = new AxisComparerY();
        static AxisComparerZ zComparer = new AxisComparerZ();

        unsafe void CentroidSort(int[] leafIds, BoundingBox[] boundingBoxes, int start, int length)
        {
            if (length == 0)
                return;
            int max = start + length;
            BoundingBox merged = boundingBoxes[start];
            for (int i = start + 1; i < max; ++i)
            {
                BoundingBox.Merge(ref merged, ref boundingBoxes[i], out merged);
            }
            var offset = merged.Max - merged.Min;
            if (offset.X > offset.Y && offset.X > offset.Z)
                Array.Sort(boundingBoxes, leafIds, start, length, xComparer);
            else if (offset.Y > offset.Z)
                Array.Sort(boundingBoxes, leafIds, start, length, yComparer);
            else
                Array.Sort(boundingBoxes, leafIds, start, length, zComparer);
        }




        unsafe void MedianSplitAddNode(int parentIndex, int indexInParent, int[] leafIds, BoundingBox[] leafBounds, int start, int length, out BoundingBox mergedBoundingBox, out int nodeIndex)
        {

            Node emptyNode;
            InitializeNode(out emptyNode);
            nodeIndex = Add(ref emptyNode); //This is a kinda stupid design! Inserting an empty node so we can go back and fill it later!
            var node = Nodes + nodeIndex;
            node->Parent = parentIndex;
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
                    MedianSplitAllocateLeafInNode(leafIds[i + start], ref boundingBoxes[i], nodeIndex, out children[i], out leafCounts[i], i);
                    BoundingBox.Merge(ref boundingBoxes[i], ref mergedBoundingBox, out mergedBoundingBox);
                }
                return;
            }
            //It is an internal node.
            //Sort it.


            var per = length / ChildrenCapacity;
            var remainder = length - ChildrenCapacity * per;
            int* lengths = stackalloc int[ChildrenCapacity];
            for (int i = 0; i < ChildrenCapacity; ++i)
            {
                int extra;
                if (per < ChildrenCapacity)
                {
                    //Put in as many leaves of the remainder as possible into single nodes to minimize the number of internal nodes.
                    //This increases occupancy.
                    if (per + remainder > ChildrenCapacity)
                        extra = ChildrenCapacity - per;
                    else
                        extra = remainder;
                    remainder -= extra;
                }
                else
                {
                    if (remainder > 0)
                    {
                        extra = 1;
                        --remainder;
                    }
                    else
                    {
                        extra = 0;
                    }
                }
                lengths[i] = per + extra;
            }

            int* starts = stackalloc int[ChildrenCapacity];
            starts[0] = start;
            for (int i = 1; i < ChildrenCapacity; ++i)
            {
                starts[i] = starts[i - 1] + lengths[i - 1];
            }

            //Node2
#if NODE2 || NODE4 || NODE8 || NODE16
            CentroidSort(leafIds, leafBounds, start, length);
#endif

            //Node4
#if NODE4 || NODE8 || NODE16
            const int halfCapacity = ChildrenCapacity / 2;
            CentroidSort(leafIds, leafBounds, start, starts[halfCapacity] - start);
            CentroidSort(leafIds, leafBounds, starts[halfCapacity], start + length - starts[halfCapacity]);
#endif

            //Node8
#if NODE8 || NODE16
            const int quarterCapacity = ChildrenCapacity / 4;
            CentroidSort(leafIds, leafBounds, start, starts[quarterCapacity] - start);
            CentroidSort(leafIds, leafBounds, starts[quarterCapacity], starts[quarterCapacity * 2] - starts[quarterCapacity]);
            CentroidSort(leafIds, leafBounds, starts[quarterCapacity * 2], starts[quarterCapacity * 3] - starts[quarterCapacity * 2]);
            CentroidSort(leafIds, leafBounds, starts[quarterCapacity * 3], start + length - starts[quarterCapacity * 3]);
#endif

            //Node16
#if NODE16
            const int eighthCapacity = ChildrenCapacity / 8;
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 0], starts[eighthCapacity * 1] - starts[eighthCapacity * 0]);
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 1], starts[eighthCapacity * 2] - starts[eighthCapacity * 1]);
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 2], starts[eighthCapacity * 3] - starts[eighthCapacity * 2]);
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 3], starts[eighthCapacity * 4] - starts[eighthCapacity * 3]);
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 4], starts[eighthCapacity * 5] - starts[eighthCapacity * 4]);
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 5], starts[eighthCapacity * 6] - starts[eighthCapacity * 5]);
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 6], starts[eighthCapacity * 7] - starts[eighthCapacity * 6]);
            CentroidSort(leafIds, leafBounds, starts[eighthCapacity * 7], start + length - starts[eighthCapacity * 7]);
#endif





            mergedBoundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < ChildrenCapacity; ++i)
            {
                if (lengths[i] == 1)
                {
                    //Stick the leaf in this slot and continue to the next child.
                    boundingBoxes[starts[i]] = leafBounds[i];
                    MedianSplitAllocateLeafInNode(leafIds[starts[i]], ref boundingBoxes[starts[i]], nodeIndex, out children[i], out leafCounts[i], i);
                    leafCounts[i] = 1;
                }
                else
                {
                    //Multiple children fit this slot. Create another internal node.
                    MedianSplitAddNode(nodeIndex, i, leafIds, leafBounds, starts[i], lengths[i], out boundingBoxes[i], out children[i]);
                    ++Nodes[nodeIndex].ChildCount;
                    leafCounts[i] = lengths[i];
                }
                BoundingBox.Merge(ref boundingBoxes[i], ref mergedBoundingBox, out mergedBoundingBox);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void MedianSplitAllocateLeafInNode(
            int leafId, ref BoundingBox leafBounds,
            int nodeIndex,
            out int nodeChild, out int leafCount, int childIndex)
        {
            var treeLeafIndex = AddLeaf(leafId, nodeIndex, childIndex);
            nodeChild = Encode(treeLeafIndex);
            ++Nodes[nodeIndex].ChildCount;
            leafCount = 1;
        }

        public unsafe void BuildMedianSplit(int[] leafIds, BoundingBox[] leafBounds, int start = 0, int length = -1)
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
            if (Nodes[0].ChildCount != 0)
                throw new InvalidOperationException("Cannot build a tree that already contains nodes.");
            //The tree is built with an empty node at the root to make insertion work more easily.
            //As long as that is the case (and as long as this is not a constructor),
            //we must clear it out.
            NodeCount = 0;

            int nodeIndex;
            BoundingBox boundingBox;
            MedianSplitAddNode(-1, -1, leafIds, leafBounds, start, length, out boundingBox, out nodeIndex);



        }
    }
}
