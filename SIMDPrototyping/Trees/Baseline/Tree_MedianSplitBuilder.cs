//#define NODE8


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
        static void ComputeCentroid(T leaf, out Vector3 centroid)
        {
            BoundingBox aabb;
            leaf.GetBoundingBox(out aabb);
            centroid = aabb.Min + aabb.Max;
        }
        //This sort could be a LOT faster if the data was provided in dedicated "leaf" form (AABB, index)
        //And may need to provide your own SIMD sort implementation...
        class AxisComparerX : IComparer<T>
        {
            public int Compare(T x, T y)
            {
                Vector3 a, b;
                ComputeCentroid(x, out a);
                ComputeCentroid(y, out b);
                return a.X.CompareTo(b.X);
            }
        }
        class AxisComparerY : IComparer<T>
        {
            public int Compare(T x, T y)
            {
                Vector3 a, b;
                ComputeCentroid(x, out a);
                ComputeCentroid(y, out b);
                return a.Y.CompareTo(b.Y);
            }
        }
        class AxisComparerZ : IComparer<T>
        {
            public int Compare(T x, T y)
            {
                Vector3 a, b;
                ComputeCentroid(x, out a);
                ComputeCentroid(y, out b);
                return a.Z.CompareTo(b.Z);
            }
        }
        static AxisComparerX xComparer = new AxisComparerX();
        static AxisComparerY yComparer = new AxisComparerY();
        static AxisComparerZ zComparer = new AxisComparerZ();

        void CentroidSort(T[] leaves, int start, int length)
        {
            if (length == 0)
                return;
            int max = start + length;
            BoundingBox merged;
            leaves[start].GetBoundingBox(out merged);
            for (int i = start + 1; i < max; ++i)
            {
                BoundingBox leafBoundingBox;
                leaves[i].GetBoundingBox(out leafBoundingBox);
                BoundingBox.Merge(ref merged, ref leafBoundingBox, out merged);
            }
            var offset = merged.Max - merged.Min;
            if (offset.X > offset.Y && offset.X > offset.Z)
                Array.Sort(leaves, start, length, xComparer);
            else if (offset.Y > offset.Z)
                Array.Sort(leaves, start, length, yComparer);
            else
                Array.Sort(leaves, start, length, zComparer);
        }




        unsafe void MedianSplitAddNode(int level, int parentIndex, int indexInParent, T[] leaves, int start, int length, out BoundingBox mergedBoundingBox, out int nodeIndex)
        {
            EnsureLevel(level);
            Node emptyNode;
            InitializeNode(out emptyNode);
            nodeIndex = Levels[level].Add(ref emptyNode); //This is a kinda stupid design! Inserting an empty node so we can go back and fill it later!
            var node = Levels[level].Nodes + nodeIndex;
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
                    MedianSplitAllocateLeafInNode(leaves[i + start], level, nodeIndex, out boundingBoxes[i], out children[i], out leafCounts[i], i);
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
            CentroidSort(leaves, start, length);
#endif

            //Node4
#if NODE4 || NODE8 || NODE16
            const int halfCapacity = ChildrenCapacity / 2;
            CentroidSort(leaves, start, starts[halfCapacity] - start);
            CentroidSort(leaves, starts[halfCapacity], start + length - starts[halfCapacity]);
#endif

            //Node8
#if NODE8 || NODE16
            const int quarterCapacity = ChildrenCapacity / 4;
            CentroidSort(leaves, start, starts[quarterCapacity] - start);
            CentroidSort(leaves, starts[quarterCapacity], starts[quarterCapacity * 2] - starts[quarterCapacity]);
            CentroidSort(leaves, starts[quarterCapacity * 2], starts[quarterCapacity * 3] - starts[quarterCapacity * 2]);
            CentroidSort(leaves, starts[quarterCapacity * 3], start + length - starts[quarterCapacity * 3]);
#endif

            //Node16
#if NODE16
            const int eighthCapacity = ChildrenCapacity / 8;
            CentroidSort(leaves, starts[eighthCapacity * 0], starts[eighthCapacity * 1] - starts[eighthCapacity * 0]);
            CentroidSort(leaves, starts[eighthCapacity * 1], starts[eighthCapacity * 2] - starts[eighthCapacity * 1]);
            CentroidSort(leaves, starts[eighthCapacity * 2], starts[eighthCapacity * 3] - starts[eighthCapacity * 2]);
            CentroidSort(leaves, starts[eighthCapacity * 3], starts[eighthCapacity * 4] - starts[eighthCapacity * 3]);
            CentroidSort(leaves, starts[eighthCapacity * 4], starts[eighthCapacity * 5] - starts[eighthCapacity * 4]);
            CentroidSort(leaves, starts[eighthCapacity * 5], starts[eighthCapacity * 6] - starts[eighthCapacity * 5]);
            CentroidSort(leaves, starts[eighthCapacity * 6], starts[eighthCapacity * 7] - starts[eighthCapacity * 6]);
            CentroidSort(leaves, starts[eighthCapacity * 7], start + length - starts[eighthCapacity * 7]);
#endif





            mergedBoundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(-float.MaxValue) };
            for (int i = 0; i < ChildrenCapacity; ++i)
            {
                if (lengths[i] == 1)
                {
                    //Stick the leaf in this slot and continue to the next child.
                    MedianSplitAllocateLeafInNode(leaves[starts[i]], level, nodeIndex, out boundingBoxes[i], out children[i], out leafCounts[i], i);
                    leafCounts[i] = 1;
                }
                else
                {
                    //Multiple children fit this slot. Create another internal node.
                    MedianSplitAddNode(level + 1, nodeIndex, i, leaves, starts[i], lengths[i], out boundingBoxes[i], out children[i]);
                    ++Levels[level].Nodes[nodeIndex].ChildCount;
                    leafCounts[i] = lengths[i];
                }
                BoundingBox.Merge(ref boundingBoxes[i], ref mergedBoundingBox, out mergedBoundingBox);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void MedianSplitAllocateLeafInNode(
            T leaf,
            int level, int nodeIndex,
            out BoundingBox boundingBox, out int nodeChild, out int leafCount, int childIndex)
        {
            leaf.GetBoundingBox(out boundingBox);
            var treeLeafIndex = AddLeaf(leaf, level, nodeIndex, childIndex);
            nodeChild = Encode(treeLeafIndex);
            ++Levels[level].Nodes[nodeIndex].ChildCount;
            leafCount = 1;
        }

        public unsafe void BuildMedianSplit(T[] leaves, int start = 0, int length = -1)
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
            MedianSplitAddNode(0, -1, -1, leaves, start, length, out boundingBox, out nodeIndex);



        }
    }
}
