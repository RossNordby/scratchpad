﻿using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;


namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        internal unsafe struct SweepResources
        {
            public BoundingBox* Bounds;
            public int* IndexMap;
            public int* IndexMapX;
            public int* IndexMapY;
            public int* IndexMapZ;
            public float* CentroidsX;
            public float* CentroidsY;
            public float* CentroidsZ;
            public BoundingBox* Merged;
        }


        unsafe void FindPartitionForAxis(BoundingBox* boundingBoxes, BoundingBox* aMerged, float* centroids, int* indexMap, int count,
            out int splitIndex, out float cost, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            Debug.Assert(count > 1);
            //TODO: Note that sorting at every level isn't necessary. Like in one of the much older spatial splitting implementations we did, you can just sort once, and thereafter
            //just do an O(n) operation to shuffle leaf data to the relevant location on each side of the partition. That allows us to punt all sort work to a prestep.
            //There, we could throw an optimized parallel sort at it. Or just do the three axes independently, hidden alongside some other work maybe.
            //I suspect the usual problems with parallel sorts would be mitigated somewhat by having three of them going on at the same time- more chances for load balancing.
            //Also note that, at each step, both the above partitioning scheme and the sort result in a contiguous block of data to work on.
            //If you're already doing a gather like that, you might as well throw wider SIMD at the problem. This version only goes up to 3 wide, which is unfortunate for AVX2 and AVX512.
            //With those changes, we can probably get the sweep builder to be faster than v1's insertion builder- it's almost there already.
            //(You'll also want to bench it against similarly simd accelerated binned approaches for use in incremental refinement. If it's not much slower, the extra quality benefits
            //might make it faster on net by virtue of speeding up self-tests, which are a dominant cost.)
            var comparer = new PrimitiveComparer<float>();
            QuickSort.Sort(ref centroids[0], ref indexMap[0], 0, count - 1, ref comparer);

            //Search for the best split.
            //Sweep across from low to high, caching the merged size and leaf count at each point.
            //Index N includes every subtree from 0 to N, inclusive. So index 0 contains subtree 0's information.
            var lastIndex = count - 1;

            aMerged[0] = boundingBoxes[indexMap[0]];
            for (int i = 1; i < lastIndex; ++i)
            {
                var index = indexMap[i];
                BoundingBox.CreateMerged(ref aMerged[i - 1], ref boundingBoxes[index], out aMerged[i]);
            }

            //Sweep from high to low.
            BoundingBox bMerged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            cost = float.MaxValue;
            splitIndex = 0;
            a = bMerged;
            b = bMerged;
            leafCountA = 0;
            leafCountB = 0;
            for (int i = lastIndex; i >= 1; --i)
            {
                int aIndex = i - 1;
                var subtreeIndex = indexMap[i];
                BoundingBox.CreateMerged(ref bMerged, ref boundingBoxes[subtreeIndex], out bMerged);

                var aCost = i * ComputeBoundsMetric(ref aMerged[aIndex]);
                var bCost = (count - i) * ComputeBoundsMetric(ref bMerged);

                var totalCost = aCost + bCost;
                if (totalCost < cost)
                {
                    cost = totalCost;
                    splitIndex = i;
                    a = aMerged[aIndex];
                    b = bMerged;
                    leafCountA = i;
                    leafCountB = count - i;
                }

            }

        }

        unsafe void FindPartition(ref SweepResources leaves, int start, int count,
               out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            //A variety of potential microoptimizations exist here.

            //Initialize the per-axis candidate maps.
            for (int i = 0; i < count; ++i)
            {
                var originalValue = leaves.IndexMap[i + start];
                leaves.IndexMapX[i] = originalValue;
                leaves.IndexMapY[i] = originalValue;
                leaves.IndexMapZ[i] = originalValue;
            }

            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsX, leaves.IndexMapX, count,
                out int xSplitIndex, out float xCost, out BoundingBox xA, out BoundingBox xB, out int xLeafCountA, out int xLeafCountB);
            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsY, leaves.IndexMapY, count,
                out int ySplitIndex, out float yCost, out BoundingBox yA, out BoundingBox yB, out int yLeafCountA, out int yLeafCountB);
            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsZ, leaves.IndexMapZ, count,
                out int zSplitIndex, out float zCost, out BoundingBox zA, out BoundingBox zB, out int zLeafCountA, out int zLeafCountB);

            int* bestIndexMap;
            if (xCost <= yCost && xCost <= zCost)
            {
                splitIndex = xSplitIndex;
                a = xA;
                b = xB;
                leafCountA = xLeafCountA;
                leafCountB = xLeafCountB;
                bestIndexMap = leaves.IndexMapX;
            }
            else if (yCost <= zCost)
            {
                splitIndex = ySplitIndex;
                a = yA;
                b = yB;
                leafCountA = yLeafCountA;
                leafCountB = yLeafCountB;
                bestIndexMap = leaves.IndexMapY;
            }
            else
            {
                splitIndex = zSplitIndex;
                a = zA;
                b = zB;
                leafCountA = zLeafCountA;
                leafCountB = zLeafCountB;
                bestIndexMap = leaves.IndexMapZ;
            }
            for (int i = 0; i < count; ++i)
            {
                leaves.IndexMap[i + start] = bestIndexMap[i];
            }

            splitIndex += start;


        }

        unsafe void SplitLeavesIntoChildren(ref SweepResources leaves, int start, int count, int nodeIndex)
        {
            if (count > 1)
            {

                FindPartition(ref leaves, start, count, out int splitIndex, out BoundingBox aBounds, out BoundingBox bBounds, out int leafCountA, out int leafCountB);

                var node = nodes + nodeIndex;
                var childIndexA = node->ChildCount++;
                var childIndexB = node->ChildCount++;
                Debug.Assert(node->ChildCount <= 2);

                ref var a = ref node->A;
                ref var b = ref node->B;
                a.Min = aBounds.Min;
                a.Max = aBounds.Max;
                b.Min = bBounds.Min;
                b.Max = bBounds.Max;

                a.LeafCount = leafCountA;
                b.LeafCount = leafCountB;

                if (leafCountA > 1)
                {
                    a.Index = CreateSweepBuilderNode(nodeIndex, childIndexA, ref leaves, start, leafCountA);
                }
                else
                {
                    Debug.Assert(leafCountA == 1);
                    //Only one leaf. Don't create another node.
                    var leafIndex = AddLeaf(nodeIndex, childIndexA);
                    a.Index = Encode(leafIndex);
                }
                if (leafCountB > 1)
                {
                    b.Index = CreateSweepBuilderNode(nodeIndex, childIndexB, ref leaves, splitIndex, leafCountB);
                }
                else
                {
                    Debug.Assert(leafCountB == 1);
                    //Only one leaf. Don't create another node.
                    var leafIndex = AddLeaf(nodeIndex, childIndexB);
                    b.Index = Encode(leafIndex);
                }
            }
            else
            {
                Debug.Assert(count == 1);
                //Only one leaf. Just stick it directly into the node.
                var node = nodes + nodeIndex;
                var childIndex = node->ChildCount++;
                Debug.Assert(node->ChildCount <= 2);
                var index = leaves.IndexMap[start];
                var leafIndex = AddLeaf(nodeIndex, childIndex);
                ref var child = ref (&node->A)[childIndex];
                child.Min = leaves.Bounds[index].Min;
                child.Max = leaves.Bounds[index].Max;
                child.Index = Encode(leafIndex);
                child.LeafCount = 1;

            }
        }

        unsafe int CreateSweepBuilderNode(int parentIndex, int indexInParent,
            ref SweepResources leaves, int start, int count)
        {
            var nodeIndex = AllocateNode();
            var node = nodes + nodeIndex;
            node->Parent = parentIndex;
            node->IndexInParent = indexInParent;

            if (count <= 2)
            {
                //No need to do any sorting. This node can fit every remaining subtree.
                node->ChildCount = count;
                var children = &node->A;
                for (int i = 0; i < count; ++i)
                {
                    var index = leaves.IndexMap[i + start];
                    var leafIndex = AddLeaf(nodeIndex, i);
                    ref var child = ref children[i];
                    child.Min = leaves.Bounds[index].Min;
                    child.Max = leaves.Bounds[index].Max;
                    child.Index = Encode(leafIndex);
                    child.LeafCount = 1;
                }
                return nodeIndex;
            }



            SplitLeavesIntoChildren(ref leaves, start, count, nodeIndex);


            return nodeIndex;

        }


        public unsafe void SweepBuild(BoundingBox[] leafBounds, int[] outputLeafIndices, int start = 0, int length = -1)
        {
            if (length == 0)
                throw new ArgumentException("Length must be positive.");
            if (length < 0)
                length = leafBounds.Length - start;
            if (start + length > outputLeafIndices.Length || start + length > leafBounds.Length)
                throw new ArgumentException("Start + length must be smaller than the leaves array lengths.");
            if (start < 0)
                throw new ArgumentException("Start must be nonnegative.");
            if (LeafCount != 0)
                throw new InvalidOperationException("Cannot build a tree that already contains nodes.");
            //The tree is built with an empty node at the root to make insertion work more easily.
            //As long as that is the case (and as long as this is not a constructor),
            //we must clear it out.
            nodeCount = 0;

            //Guarantee that no resizes will occur during the build.
            if (Leaves.Length < length)
            {
                Resize(leafBounds.Length);
            }

            //Gather necessary information and put it into a convenient format.

            var intPool = Pool.SpecializeFor<int>();
            var floatPool = Pool.SpecializeFor<float>();
            intPool.Take(length, out var indexMapX);
            intPool.Take(length, out var indexMapY);
            intPool.Take(length, out var indexMapZ);
            floatPool.Take(length, out var centroidsX);
            floatPool.Take(length, out var centroidsY);
            floatPool.Take(length, out var centroidsZ);
            Pool.SpecializeFor<BoundingBox>().Take(length, out var merged);
            //TODO: Would ideally use spans here, rather than assuming managed input. We could check generic type parameters to efficiently get pointers out.
            fixed (BoundingBox* leafBoundsPointer = &leafBounds[start])
            fixed (int* indexMapPointer = &outputLeafIndices[start])
            {
                SweepResources leaves;
                leaves.Bounds = leafBoundsPointer;
                leaves.IndexMap = indexMapPointer;
                leaves.IndexMapX = (int*)indexMapX.Memory;
                leaves.IndexMapY = (int*)indexMapY.Memory;
                leaves.IndexMapZ = (int*)indexMapZ.Memory;
                leaves.CentroidsX = (float*)centroidsX.Memory;
                leaves.CentroidsY = (float*)centroidsY.Memory;
                leaves.CentroidsZ = (float*)centroidsZ.Memory;
                leaves.Merged = (BoundingBox*)merged.Memory;

                for (int i = 0; i < length; ++i)
                {
                    var bounds = leaves.Bounds[i];
                    leaves.IndexMap[i] = i;
                    //Per-axis index maps don't need to be initialized here. They're filled in at the time of use.

                    var centroid = bounds.Min + bounds.Max;
                    centroidsX[i] = centroid.X;
                    centroidsY[i] = centroid.Y;
                    centroidsZ[i] = centroid.Z;
                }

                //Now perform a top-down sweep build.
                CreateSweepBuilderNode(-1, -1, ref leaves, 0, length);

            }


            //Return resources.            
            floatPool.Return(ref centroidsX);
            floatPool.Return(ref centroidsY);
            floatPool.Return(ref centroidsZ);            
            intPool.Return(ref indexMapX);
            intPool.Return(ref indexMapY);
            intPool.Return(ref indexMapZ);
            Pool.SpecializeFor<BoundingBox>().Return(ref merged);

        }
    }
}
