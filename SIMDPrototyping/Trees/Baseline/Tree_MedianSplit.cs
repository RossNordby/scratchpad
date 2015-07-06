#define NODE8


using System;
using System.Collections.Generic;
using System.Linq;
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
        //This sort could be a LOT faster if the data was provided in dedicated "leaf" form (AABB, index)
        //And may need to provide your own SIMD sort implementation...
        class AxisComparerX : IComparer<T>
        {
            public int Compare(T x, T y)
            {
                BoundingBox a, b;
                x.GetBoundingBox(out a);
                y.GetBoundingBox(out b);
                return a.Min.X.CompareTo(b.Min.X);
            }
        }
        class AxisComparerY : IComparer<T>
        {
            public int Compare(T x, T y)
            {
                BoundingBox a, b;
                x.GetBoundingBox(out a);
                y.GetBoundingBox(out b);
                return a.Min.Y.CompareTo(b.Min.Y);
            }
        }
        class AxisComparerZ : IComparer<T>
        {
            public int Compare(T x, T y)
            {
                BoundingBox a, b;
                x.GetBoundingBox(out a);
                y.GetBoundingBox(out b);
                return a.Min.Z.CompareTo(b.Min.Z);
            }
        }
        static AxisComparerX xComparer = new AxisComparerX();
        static AxisComparerY yComparer = new AxisComparerY();
        static AxisComparerZ zComparer = new AxisComparerZ();

        void Sort(T[] leaves, int start, int length)
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

        unsafe void MedianSplitAddNode(int level, T[] leaves, int start, int length)
        {
            EnsureLevel(level);
            Node node;
            InitializeNode(out node);
            var nodeIndex = Levels[level].Add(ref node); //This is a kinda stupid design! Inserting an empty node so we can go back and fill it later!
            var boundingBoxes = &Levels[level].Nodes[nodeIndex].A;
            var children = &Levels[level].Nodes[nodeIndex].ChildA;

            if (length <= ChildrenCapacity)
            {
                //Don't need to do any sorting at all. This internal node contains only leaves.                
                for (int i = 0; i < length; ++i)
                {
                    MedianSplitAllocateLeafInNode(leaves[i + start], level, nodeIndex, boundingBoxes, children, i);

                }
                return;
            }
            //It is an internal node.
            //Sort it.

            //Node2
            Sort(leaves, start, length);

            //Node4
            int half = start + length / 2;
            Sort(leaves, start, half - start);
            Sort(leaves, half, length - half);

            //Node8
            int oneQuarter = start + (half - start) / 2;
            int threeQuarters = half + (length - half) / 2;
            Sort(leaves, start, oneQuarter - start);
            Sort(leaves, oneQuarter, half - oneQuarter);
            Sort(leaves, half, threeQuarters - half);
            Sort(leaves, threeQuarters, length - threeQuarters);

            int* octStarts = stackalloc int[8];
            octStarts[0] = start;
            octStarts[1] = start + (oneQuarter - start) / 2;
            octStarts[2] = oneQuarter;
            octStarts[3] = octStarts[2] + (half - oneQuarter) / 2;
            octStarts[4] = half;
            octStarts[5] = octStarts[4] + (threeQuarters - octStarts[4]) / 2;
            octStarts[6] = threeQuarters;
            octStarts[7] = octStarts[6] + (length - octStarts[6]) / 2;
            //There are eight segments.
            int* octLengths = stackalloc int[8];
            
            for (int i = 0; i < 7; ++i)
            {
                octLengths[i] = octStarts[i + 1] - octStarts[i];
            }
            octLengths[7] = length - octStarts[7];

            for (int i = 0; i < 8; ++i)
            {
                if (octLengths[i] == 1)
                {
                    //Stick the leaf in this slot and continue to the next child.
                    MedianSplitAllocateLeafInNode(leaves[octStarts[i]], level, nodeIndex, boundingBoxes, children, i);
                    continue;
                }
                //Multiple children fit this slot. Create another internal node.
                MedianSplitAddNode(level + 1, leaves, octStarts[i], octLengths[i]);
            }

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void MedianSplitAllocateLeafInNode(
            T leaf,
            int level, int nodeIndex,
            BoundingBox* boundingBoxes, int* children, int childIndex)
        {
            leaf.GetBoundingBox(out boundingBoxes[childIndex]);
            var treeLeafIndex = AddLeaf(leaf, level, nodeIndex, childIndex);
            children[childIndex] = Encode(treeLeafIndex);
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
            
            MedianSplitAddNode(0, leaves, start, length);



        }
    }
}
