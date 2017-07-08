//#define NODE8


using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;


namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        internal unsafe struct SweepResources
        {
            public BoundingBox* Bounds;
            public int* Ids;
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


            Quicksort(centroids, indexMap, 0, count - 1);

            //Search for the best split.
            //Sweep across from low to high, caching the merged size and leaf count at each point.
            //Index N includes every subtree from 0 to N, inclusive. So index 0 contains subtree 0's information.
            var lastIndex = count - 1;

            aMerged[0] = boundingBoxes[indexMap[0]];
            for (int i = 1; i < lastIndex; ++i)
            {
                var index = indexMap[i];
                BoundingBox.Merge(ref aMerged[i - 1], ref boundingBoxes[index], out aMerged[i]);
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
                BoundingBox.Merge(ref bMerged, ref boundingBoxes[subtreeIndex], out bMerged);

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

            int xSplitIndex, xLeafCountA, xLeafCountB, ySplitIndex, yLeafCountA, yLeafCountB, zSplitIndex, zLeafCountA, zLeafCountB;
            BoundingBox xA, xB, yA, yB, zA, zB;
            float xCost, yCost, zCost;
            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsX, leaves.IndexMapX, count, out xSplitIndex, out xCost, out xA, out xB, out xLeafCountA, out xLeafCountB);
            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsY, leaves.IndexMapY, count, out ySplitIndex, out yCost, out yA, out yB, out yLeafCountA, out yLeafCountB);
            FindPartitionForAxis(leaves.Bounds, leaves.Merged, leaves.CentroidsZ, leaves.IndexMapZ, count, out zSplitIndex, out zCost, out zA, out zB, out zLeafCountA, out zLeafCountB);

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

        unsafe void SplitLeavesIntoChildren(int depthRemaining, ref SweepResources leaves,
            int start, int count, int nodeIndex)
        {
            if (count > 1)
            {

                BoundingBox a, b;
                int leafCountA, leafCountB;
                int splitIndex;
                FindPartition(ref leaves, start, count, out splitIndex, out a, out b, out leafCountA, out leafCountB);

                if (depthRemaining > 0)
                {
                    --depthRemaining;
                    SplitLeavesIntoChildren(depthRemaining, ref leaves, start, splitIndex - start, nodeIndex);
                    SplitLeavesIntoChildren(depthRemaining, ref leaves, splitIndex, start + count - splitIndex, nodeIndex);
                }
                else
                {
                    //Recursion bottomed out. 
                    var node = nodes + nodeIndex;
                    var childIndexA = node->ChildCount++;
                    var childIndexB = node->ChildCount++;
                    Debug.Assert(node->ChildCount <= ChildrenCapacity);

                    var bounds = &node->A;
                    var children = &node->ChildA;
                    var leafCounts = &node->LeafCountA;

                    bounds[childIndexA] = a;
                    bounds[childIndexB] = b;

                    leafCounts[childIndexA] = leafCountA;
                    leafCounts[childIndexB] = leafCountB;

                    if (leafCountA > 1)
                    {
                        children[childIndexA] = CreateSweepBuilderNode(nodeIndex, childIndexA, ref leaves, start, leafCountA);
                    }
                    else
                    {
                        Debug.Assert(leafCountA == 1);
                        //Only one leaf. Don't create another node.
                        bool leavesInvalidated;
                        var leafIndex = AddLeaf(leaves.Ids[leaves.IndexMap[start]], nodeIndex, childIndexA, out leavesInvalidated);
                        Debug.Assert(!leavesInvalidated);
                        children[childIndexA] = Encode(leafIndex);
                    }
                    if (leafCountB > 1)
                    {
                        children[childIndexB] = CreateSweepBuilderNode(nodeIndex, childIndexB, ref leaves, splitIndex, leafCountB);
                    }
                    else
                    {
                        Debug.Assert(leafCountB == 1);
                        //Only one leaf. Don't create another node.
                        bool leavesInvalidated;
                        var leafIndex = AddLeaf(leaves.Ids[leaves.IndexMap[splitIndex]], nodeIndex, childIndexB, out leavesInvalidated);
                        Debug.Assert(!leavesInvalidated);
                        children[childIndexB] = Encode(leafIndex);
                    }
                }
            }
            else
            {
                Debug.Assert(count == 1);
                //Only one leaf. Just stick it directly into the node.
                var node = nodes + nodeIndex;
                var childIndex = node->ChildCount++;
                Debug.Assert(node->ChildCount <= ChildrenCapacity);
                bool leavesInvalidated;
                var index = leaves.IndexMap[start];
                var leafIndex = AddLeaf(leaves.Ids[index], nodeIndex, childIndex, out leavesInvalidated);
                Debug.Assert(!leavesInvalidated);
                (&node->A)[childIndex] = leaves.Bounds[index];
                (&node->ChildA)[childIndex] = Encode(leafIndex);
                (&node->LeafCountA)[childIndex] = 1;

            }
        }

        unsafe int CreateSweepBuilderNode(int parentIndex, int indexInParent,
            ref SweepResources leaves, int start, int count)
        {
            bool nodesInvalidated;
            var nodeIndex = AllocateNode(out nodesInvalidated);
            Debug.Assert(!nodesInvalidated, "The sweep builder should have allocated enough nodes ahead of time.");
            var node = nodes + nodeIndex;
            node->Parent = parentIndex;
            node->IndexInParent = indexInParent;

            if (count <= ChildrenCapacity)
            {
                //No need to do any sorting. This node can fit every remaining subtree.
                node->ChildCount = count;
                var bounds = &node->A;
                var children = &node->ChildA;
                var leafCounts = &node->LeafCountA;
                for (int i = 0; i < count; ++i)
                {
                    var index = leaves.IndexMap[i + start];
                    bool leavesInvalidated;
                    var leafIndex = AddLeaf(leaves.Ids[index], nodeIndex, i, out leavesInvalidated);
                    Debug.Assert(!leavesInvalidated);
                    bounds[i] = leaves.Bounds[index];
                    children[i] = Encode(leafIndex);
                    leafCounts[i] = 1;
                }
                return nodeIndex;
            }

            const int recursionDepth = ChildrenCapacity == 32 ? 4 : ChildrenCapacity == 16 ? 3 : ChildrenCapacity == 8 ? 2 : ChildrenCapacity == 4 ? 1 : 0;


            SplitLeavesIntoChildren(recursionDepth, ref leaves, start, count, nodeIndex);


            return nodeIndex;

        }


        public unsafe void SweepBuild(int[] leafIds, BoundingBox[] leafBounds, int start = 0, int length = -1, BufferPool<int> intPool = null, BufferPool<float> floatPool = null)
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

            //Gather necessary information and put it into a convenient format.

            var indexMap = intPool == null ? new int[leafIds.Length] : intPool.Take(leafIds.Length);
            var indexMapX = intPool == null ? new int[leafIds.Length] : intPool.Take(leafIds.Length);
            var indexMapY = intPool == null ? new int[leafIds.Length] : intPool.Take(leafIds.Length);
            var indexMapZ = intPool == null ? new int[leafIds.Length] : intPool.Take(leafIds.Length);
            var centroidsX = floatPool == null ? new float[leafIds.Length] : floatPool.Take(leafIds.Length);
            var centroidsY = floatPool == null ? new float[leafIds.Length] : floatPool.Take(leafIds.Length);
            var centroidsZ = floatPool == null ? new float[leafIds.Length] : floatPool.Take(leafIds.Length);
            var mergedSize = leafIds.Length * (sizeof(BoundingBox) / sizeof(float));
            var merged = floatPool == null ? new float[mergedSize] : floatPool.Take(mergedSize);
            fixed (BoundingBox* leafBoundsPointer = leafBounds)
            fixed (int* leafIdsPointer = leafIds)
            fixed (int* indexMapPointer = indexMap)
            fixed (int* indexMapXPointer = indexMapX)
            fixed (int* indexMapYPointer = indexMapY)
            fixed (int* indexMapZPointer = indexMapZ)
            fixed (float* centroidsXPointer = centroidsX)
            fixed (float* centroidsYPointer = centroidsY)
            fixed (float* centroidsZPointer = centroidsZ)
            fixed (float* mergedPointer = merged)
            {
                SweepResources leaves;
                leaves.Bounds = leafBoundsPointer;
                leaves.Ids = leafIdsPointer;
                leaves.IndexMap = indexMapPointer;
                leaves.IndexMapX = indexMapXPointer;
                leaves.IndexMapY = indexMapYPointer;
                leaves.IndexMapZ = indexMapZPointer;
                leaves.CentroidsX = centroidsXPointer;
                leaves.CentroidsY = centroidsYPointer;
                leaves.CentroidsZ = centroidsZPointer;
                leaves.Merged = (BoundingBox*)mergedPointer;

                for (int i = 0; i < leafIds.Length; ++i)
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
                CreateSweepBuilderNode(-1, -1, ref leaves, 0, leafIds.Length);

            }


            //Return resources.
            if (floatPool != null)
            {
                Array.Clear(centroidsX, 0, leafIds.Length);
                Array.Clear(centroidsY, 0, leafIds.Length);
                Array.Clear(centroidsZ, 0, leafIds.Length);
                Array.Clear(merged, 0, mergedSize);
                floatPool.GiveBack(centroidsX);
                floatPool.GiveBack(centroidsY);
                floatPool.GiveBack(centroidsZ);
                floatPool.GiveBack(merged);
            }
            if (intPool != null)
            {
                Array.Clear(indexMap, 0, leafIds.Length);
                Array.Clear(indexMapX, 0, leafIds.Length);
                Array.Clear(indexMapY, 0, leafIds.Length);
                Array.Clear(indexMapZ, 0, leafIds.Length);
                intPool.GiveBack(indexMap);
                intPool.GiveBack(indexMapX);
                intPool.GiveBack(indexMapY);
                intPool.GiveBack(indexMapZ);
            }

        }
    }
}
