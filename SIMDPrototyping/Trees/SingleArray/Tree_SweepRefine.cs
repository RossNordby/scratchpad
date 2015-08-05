using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
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
        unsafe void Swap(ref int a, ref int b)
        {
            var temp = a;
            a = b;
            b = temp;
        }
        unsafe void Quicksort(float* centroids, int* indexMap, int l, int r)
        {
            if (r - l <= 30)
            {
                //The area to address is very small. Use insertion sort.
                for (int i = l + 1; i <= r; ++i)
                {
                    var index = i;
                    var previousIndex = index - 1;
                    while (centroids[indexMap[index]] < centroids[indexMap[previousIndex]])
                    {

                        var tempPointer = indexMap[index];
                        indexMap[index] = indexMap[previousIndex];
                        indexMap[previousIndex] = tempPointer;


                        if (previousIndex == l)
                            break;
                        index = previousIndex;
                        --previousIndex;
                    }
                }
            }
            else
            {
                //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
                var first = centroids[indexMap[l]];
                int middleIndex = (l + r) / 2;
                var middle = centroids[indexMap[middleIndex]];
                var last = centroids[indexMap[r]];
                int pivotIndex;
                float pivot;
                if (first <= middle && first <= last)
                {
                    //first is lowest.
                    if (middle <= last)
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else if (middle <= last)
                {
                    //middle is lowest.
                    if (first <= last)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else
                {
                    //last is lowest.
                    if (first <= middle)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                }

                ////Choose the middle index as the pivot, since there's a high chance of sortedness.
                //int pivotIndex = (l + r) / 2;
                //float pivot = centroids[indexMap[pivotIndex]];

                //Put the pivot into the last slot.
                Swap(ref indexMap[pivotIndex], ref indexMap[r]);

                //Use bentley-mcilroy 3-way partitioning scheme to avoid performance drops in corner cases.
                int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
                int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
                int p = l - 1;
                int q = r;
                if (r <= l)
                    return;
                while (true)
                {
                    //Claim the chunk of the list which is partitioned on the left and right sides.
                    while (centroids[indexMap[++i]] < pivot) ;
                    while (pivot < centroids[indexMap[--j]])
                    {
                        if (j == l)
                        {
                            break;
                        }
                    }
                    //If the claims have met, then the partition is complete.
                    if (i >= j)
                    {
                        break;
                    }
                    //By the claiming above and because we did not yet break out of the loop,
                    //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
                    //So swap them.
                    Swap(ref indexMap[i], ref indexMap[j]);
                    if (centroids[indexMap[i]] == pivot)
                    {
                        p++;
                        Swap(ref indexMap[p], ref indexMap[i]);
                    }
                    if (pivot == centroids[indexMap[j]])
                    {
                        q--;
                        Swap(ref indexMap[j], ref indexMap[q]);
                    }
                }
                //The pivot at r has not been swapped.
                //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
                //So, swap the pivot and the first element of the greater-than-pivot side to guarantee sorting.
                Swap(ref indexMap[i], ref indexMap[r]);
                j = i - 1;
                i = i + 1;
                for (int k = l; k < p; k++, j--)
                {
                    Swap(ref indexMap[k], ref indexMap[j]);
                }
                for (int k = r - 1; k > q; k--, i++)
                {
                    Swap(ref indexMap[i], ref indexMap[k]);
                }
                Quicksort(centroids, indexMap, l, j);
                Quicksort(centroids, indexMap, i, r);
            }
        }


        unsafe void FindPartitionForAxis(BoundingBox* boundingBoxes, int* leafCounts, float* centroids, int* indexMap, int subtreeCount,
            out int splitIndex, out float cost, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            Debug.Assert(subtreeCount > 1);

            //Sort the index map according to the centroids.
            //for (int i = 1; i < subtreeCount; ++i)
            //{
            //    var index = i;
            //    var previousIndex = index - 1;
            //    while (centroids[indexMap[index]] < centroids[indexMap[previousIndex]])
            //    {

            //        var tempPointer = indexMap[index];
            //        indexMap[index] = indexMap[previousIndex];
            //        indexMap[previousIndex] = tempPointer;


            //        if (previousIndex == 0)
            //            break;
            //        index = previousIndex;
            //        --previousIndex;
            //    }
            //}
            Quicksort(centroids, indexMap, 0, subtreeCount - 1);


            //for (int i = 1; i < subtreeCount; ++i)
            //{
            //    if (centroids[indexMap[i]] < centroids[indexMap[i - 1]])
            //    {
            //        Console.WriteLine("not sorteD");
            //    }
            //}

            //Search for the best split.
            //Sweep across from low to high, caching the merged size and leaf count at each point.
            //Index N includes every subtree from 0 to N, inclusive. So index 0 contains subtree 0's information.
            var lastIndex = subtreeCount - 1;
            var aLeafCounts = stackalloc int[lastIndex];
            var aMerged = stackalloc BoundingBox[lastIndex];

            *aLeafCounts = leafCounts[*indexMap];
            *aMerged = boundingBoxes[*indexMap];
            for (int i = 1; i < lastIndex; ++i)
            {
                var index = indexMap[i];
                aLeafCounts[i] = leafCounts[index] + aLeafCounts[i - 1];
                BoundingBox.Merge(ref aMerged[i - 1], ref boundingBoxes[index], out aMerged[i]);
            }

            //Sweep from high to low.
            BoundingBox bMerged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            cost = float.MaxValue;
            splitIndex = 0;
            int bLeafCount = 0;
            a = bMerged;
            b = bMerged;
            leafCountA = 0;
            leafCountB = 0;
            for (int i = lastIndex; i >= 1; --i)
            {
                int aIndex = i - 1;
                var subtreeIndex = indexMap[i];
                BoundingBox.Merge(ref bMerged, ref boundingBoxes[subtreeIndex], out bMerged);
                bLeafCount += leafCounts[subtreeIndex];

                var aCost = aLeafCounts[aIndex] * ComputeBoundsMetric(ref aMerged[aIndex]);
                var bCost = bLeafCount * ComputeBoundsMetric(ref bMerged);

                var totalCost = aCost + bCost;
                if (totalCost < cost)
                {
                    cost = totalCost;
                    splitIndex = i;
                    a = aMerged[aIndex];
                    b = bMerged;
                    leafCountA = aLeafCounts[aIndex];
                    leafCountB = bLeafCount;
                }

            }

        }
        unsafe void FindPartition(ref Subtrees subtrees, int start, int count,
               out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            //A variety of potential microoptimizations exist here.
            //Don't reallocate centroids (because JIT is forced to zero by roslyn), do swaps better, etc.
            var indexMapX = stackalloc int[count];
            var indexMapY = stackalloc int[count];
            var indexMapZ = stackalloc int[count];

            //Initialize the per-axis candidate maps.
            var localIndexMap = subtrees.IndexMap + start;
            for (int i = 0; i < count; ++i)
            {
                var originalValue = localIndexMap[i];
                indexMapX[i] = originalValue;
                indexMapY[i] = originalValue;
                indexMapZ[i] = originalValue;
            }

            int xSplitIndex, xLeafCountA, xLeafCountB, ySplitIndex, yLeafCountA, yLeafCountB, zSplitIndex, zLeafCountA, zLeafCountB;
            BoundingBox xA, xB, yA, yB, zA, zB;
            float xCost, yCost, zCost;
            FindPartitionForAxis(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsX, indexMapX, count, out xSplitIndex, out xCost, out xA, out xB, out xLeafCountA, out xLeafCountB);
            FindPartitionForAxis(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsY, indexMapY, count, out ySplitIndex, out yCost, out yA, out yB, out yLeafCountA, out yLeafCountB);
            FindPartitionForAxis(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsZ, indexMapZ, count, out zSplitIndex, out zCost, out zA, out zB, out zLeafCountA, out zLeafCountB);

            int* bestIndexMap;
            if (xCost <= yCost && xCost <= zCost)
            {
                splitIndex = xSplitIndex;
                a = xA;
                b = xB;
                leafCountA = xLeafCountA;
                leafCountB = xLeafCountB;
                bestIndexMap = indexMapX;
            }
            else if (yCost <= zCost)
            {
                splitIndex = ySplitIndex;
                a = yA;
                b = yB;
                leafCountA = yLeafCountA;
                leafCountB = yLeafCountB;
                bestIndexMap = indexMapY;
            }
            else
            {
                splitIndex = zSplitIndex;
                a = zA;
                b = zB;
                leafCountA = zLeafCountA;
                leafCountB = zLeafCountB;
                bestIndexMap = indexMapZ;
            }
            for (int i = 0; i < count; ++i)
            {
                localIndexMap[i] = bestIndexMap[i];
            }

            splitIndex += start;


        }



        unsafe void SplitSubtreesIntoChildren(int depthRemaining, ref Subtrees subtrees,
            int start, int count, ref BoundingBox boundingBox,
            Node* stagingNodes, int stagingNodeIndex, ref int stagingNodesCount, out float childrenTreeletsCost)
        {
            if (count > 1)
            {

                BoundingBox a, b;
                int leafCountA, leafCountB;
                int splitIndex;
                FindPartition(ref subtrees, start, count, out splitIndex, out a, out b, out leafCountA, out leafCountB);

                float costA, costB;
                if (depthRemaining > 0)
                {
                    --depthRemaining;
                    SplitSubtreesIntoChildren(depthRemaining, ref subtrees, start, splitIndex - start, ref a, stagingNodes, stagingNodeIndex, ref stagingNodesCount, out costA);
                    SplitSubtreesIntoChildren(depthRemaining, ref subtrees, splitIndex, start + count - splitIndex, ref b, stagingNodes, stagingNodeIndex, ref stagingNodesCount, out costB);
                }
                else
                {
                    //Recursion bottomed out. 
                    var stagingNode = stagingNodes + stagingNodeIndex;
                    var childIndexA = stagingNode->ChildCount++;
                    var childIndexB = stagingNode->ChildCount++;
                    Debug.Assert(stagingNode->ChildCount <= ChildrenCapacity);

                    var stagingBounds = &stagingNode->A;
                    var stagingChildren = &stagingNode->ChildA;
                    var stagingLeafCounts = &stagingNode->LeafCountA;

                    stagingBounds[childIndexA] = a;
                    stagingBounds[childIndexB] = b;

                    stagingLeafCounts[childIndexA] = leafCountA;
                    stagingLeafCounts[childIndexB] = leafCountB;

                    int subtreeCountA = splitIndex - start;
                    int subtreeCountB = start + count - splitIndex;
                    if (subtreeCountA > 1)
                    {
                        stagingChildren[childIndexA] = CreateStagingNode(stagingNodeIndex, childIndexA, ref a, ref subtrees, start, subtreeCountA,
                            stagingNodes, ref stagingNodesCount, out costA);
                        costA += ComputeBoundsMetric(ref a); //An internal node was created; measure its cost.
                    }
                    else
                    {
                        Debug.Assert(subtreeCountA == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexA] = Encode(subtrees.IndexMap[start]);
                        costA = 0;
                    }
                    if (subtreeCountB > 1)
                    {
                        stagingChildren[childIndexB] = CreateStagingNode(stagingNodeIndex, childIndexB, ref b, ref subtrees, splitIndex, subtreeCountB,
                            stagingNodes, ref stagingNodesCount, out costB);
                        costB += ComputeBoundsMetric(ref b); //An internal node was created; measure its cost.
                    }
                    else
                    {
                        Debug.Assert(subtreeCountB == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexB] = Encode(subtrees.IndexMap[splitIndex]);
                        costB = 0;
                    }
                }
                childrenTreeletsCost = costA + costB;
            }
            else
            {
                Debug.Assert(count == 1);
                //Only one subtree. Just stick it directly into the node.
                var childIndex = stagingNodes[stagingNodeIndex].ChildCount++;
                var subtreeIndex = subtrees.IndexMap[start];
                Debug.Assert(stagingNodes[stagingNodeIndex].ChildCount <= ChildrenCapacity);
                (&stagingNodes[stagingNodeIndex].A)[childIndex] = subtrees.BoundingBoxes[subtreeIndex];
                (&stagingNodes[stagingNodeIndex].ChildA)[childIndex] = Encode(subtreeIndex);
                (&stagingNodes[stagingNodeIndex].LeafCountA)[childIndex] = subtrees.LeafCounts[subtreeIndex];
                //Subtrees cannot contribute to change in cost.
                childrenTreeletsCost = 0;
            }
        }

        unsafe int CreateStagingNode(int parentIndex, int indexInParent, ref BoundingBox boundingBox,
            ref Subtrees subtrees, int start, int count,
            Node* stagingNodes, ref int stagingNodeCount, out float childTreeletsCost)
        {
            var stagingNodeIndex = stagingNodeCount++;
            var stagingNode = stagingNodes + stagingNodeIndex;

            if (count <= ChildrenCapacity)
            {
                //No need to do any sorting. This node can fit every remaining subtree.
                var localIndexMap = subtrees.IndexMap + start;
                stagingNode->ChildCount = count;
                var stagingNodeBounds = &stagingNode->A;
                var stagingNodeChildren = &stagingNode->ChildA;
                var leafCounts = &stagingNode->LeafCountA;
                for (int i = 0; i < count; ++i)
                {
                    var subtreeIndex = localIndexMap[i];
                    stagingNodeBounds[i] = subtrees.BoundingBoxes[subtreeIndex];
                    leafCounts[i] = subtrees.LeafCounts[subtreeIndex];
                    stagingNodeChildren[i] = Encode(subtreeIndex);
                }
                //Because subtrees do not change in size, they cannot change the cost.
                childTreeletsCost = 0;
                return stagingNodeIndex;
            }

            const int recursionDepth = ChildrenCapacity == 32 ? 4 : ChildrenCapacity == 16 ? 3 : ChildrenCapacity == 8 ? 2 : ChildrenCapacity == 4 ? 1 : 0;


            SplitSubtreesIntoChildren(recursionDepth, ref subtrees, start, count, ref boundingBox, stagingNodes, stagingNodeIndex, ref stagingNodeCount, out childTreeletsCost);


            return stagingNodeIndex;

        }




        public unsafe void SweepRefine(int nodeIndex, ref QuickQueue<int> spareNodes, out bool nodesInvalidated)
        {
            const int maximumSubtrees = 1024;
            var poolIndex = BufferPool<int>.GetPoolIndex(maximumSubtrees);
            var subtreeReferences = new QuickList<int>(BufferPools<int>.Thread, poolIndex);
            var treeletInternalNodes = new QuickQueue<int>(BufferPools<int>.Thread, poolIndex);
            float originalTreeletCost;
            CollectSubtrees(nodeIndex, maximumSubtrees, ref subtreeReferences, ref treeletInternalNodes, out originalTreeletCost);


            //Gather necessary information from nodes.
            //Some awkwardness: can't stackalloc into anything but a local.
            var boundingBoxes = stackalloc BoundingBox[subtreeReferences.Count];
            var leafCounts = stackalloc int[subtreeReferences.Count];
            var indexMap = stackalloc int[subtreeReferences.Count];
            float* centroidsX = stackalloc float[subtreeReferences.Count];
            float* centroidsY = stackalloc float[subtreeReferences.Count];
            float* centroidsZ = stackalloc float[subtreeReferences.Count];
            Subtrees subtrees;
            subtrees.BoundingBoxes = boundingBoxes;
            subtrees.LeafCounts = leafCounts;
            subtrees.IndexMap = indexMap;
            subtrees.CentroidsX = centroidsX;
            subtrees.CentroidsY = centroidsY;
            subtrees.CentroidsZ = centroidsZ;

            for (int i = 0; i < subtreeReferences.Count; ++i)
            {
                var boundingBox = boundingBoxes + i;
                indexMap[i] = i;
                if (subtreeReferences.Elements[i] >= 0)
                {
                    //It's an internal node.
                    var subtreeNode = nodes + subtreeReferences.Elements[i];
                    var parentNode = nodes + subtreeNode->Parent;
                    *boundingBox = (&parentNode->A)[subtreeNode->IndexInParent];
                    var centroid = boundingBox->Min + boundingBox->Max;
                    centroidsX[i] = centroid.X;
                    centroidsY[i] = centroid.Y;
                    centroidsZ[i] = centroid.Z;
                    leafCounts[i] = (&parentNode->LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node.
                    var leaf = leaves + Encode(subtreeReferences.Elements[i]);
                    *boundingBox = (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                    var centroid = boundingBox->Min + boundingBox->Max;
                    centroidsX[i] = centroid.X;
                    centroidsY[i] = centroid.Y;
                    centroidsZ[i] = centroid.Z;
                    leafCounts[i] = 1;
                }
            }

            var node = nodes + nodeIndex;
            int parent = node->Parent;
            int indexInParent = node->IndexInParent;

            //Now perform a top-down sweep build.
            //TODO: this staging creation section is really the only part that is sweep-specific. The rest is common to any other kind of subtree-collection based refinement. 
            //If you end up making others, keep this in mind.
            int stagingNodeCount = 0;
            int stagingNodeCapacity = subtreeReferences.Count - 1;
            var stagingNodes = stackalloc Node[stagingNodeCapacity];

            BoundingBox treeletBoundingBox;
            if (parent >= 0)
            {
                //This node is not the root, so we can look for the bounding box in the parent node.
                treeletBoundingBox = (&nodes[parent].A)[indexInParent];
            }
            else
            {
                //This node is the root, so the bounding box must be derived.
                treeletBoundingBox = node->A;
                for (int i = 1; i < node->ChildCount; ++i)
                {
                    BoundingBox.Merge(ref treeletBoundingBox, ref (&node->A)[i], out treeletBoundingBox);
                }
            }
            float newTreeletCost;
            CreateStagingNode(parent, indexInParent, ref treeletBoundingBox, ref subtrees, 0, subtreeReferences.Count, stagingNodes, ref stagingNodeCount, out newTreeletCost);

            //ValidateStaging(stagingNodes, sweepSubtrees, ref subtreeReferences, parent, indexInParent);

            if (newTreeletCost < originalTreeletCost)
            {
                //The refinement is an actual improvement.
                //Apply the staged nodes to real nodes!
                var reifiedIndex = ReifyStagingNode(parent, indexInParent, stagingNodes, 0, stagingNodeCapacity, ref subtreeReferences, ref treeletInternalNodes, ref spareNodes, out nodesInvalidated);
                Debug.Assert(parent != -1 ? (&nodes[parent].ChildA)[indexInParent] == reifiedIndex : true, "The parent should agree with the child about the relationship.");
                //If any nodes are left over, put them into the spares list for later reuse.
                int spareNode;
                while (treeletInternalNodes.TryDequeue(out spareNode))
                {
                    spareNodes.Enqueue(spareNode);
                }
            }
            else
            {
                nodesInvalidated = false;
            }

            subtreeReferences.Dispose();
            treeletInternalNodes.Dispose();

        }





        private unsafe void TopDownSweepRefine(int nodeIndex, ref QuickQueue<int> spareNodes)
        {
            bool nodesInvalidated;
            //Validate();
            SweepRefine(nodeIndex, ref spareNodes, out nodesInvalidated);
            //Validate();
            //The root of the tree is guaranteed to stay in position, so nodeIndex is still valid.

            //Node pointers can be invalidated, so don't hold a reference between executions.
            for (int i = 0; i < nodes[nodeIndex].ChildCount; ++i)
            {
                var child = (&nodes[nodeIndex].ChildA)[i];
                if (child >= 0)
                {
                    TopDownSweepRefine(child, ref spareNodes);
                }
            }
        }
        public unsafe void TopDownSweepRefine()
        {
            var spareNodes = new QuickQueue<int>(BufferPools<int>.Thread, 8);
            TopDownSweepRefine(0, ref spareNodes);
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }


        unsafe void TryToBottomUpSweepRefine(int[] refinementFlags, int nodeIndex, ref QuickQueue<int> spareInternalNodes)
        {
            if (++refinementFlags[nodeIndex] == nodes[nodeIndex].ChildCount)
            {
                bool nodesInvalidated;
                SweepRefine(nodeIndex, ref spareInternalNodes, out nodesInvalidated);

                var parent = nodes[nodeIndex].Parent;
                if (parent != -1)
                {
                    TryToBottomUpSweepRefine(refinementFlags, parent, ref spareInternalNodes);
                }
            }
        }


        /// <summary>
        /// Executes one pass of bottom-up refinement.
        /// </summary>
        public unsafe void BottomUpSweepRefine()
        {
            //If this works out, should probably choose a more efficient flagging approach.
            //Note the size: it needs to contain all possible internal nodes.
            //TODO: This is actually bugged, because the refinement flags do not update if the nodes move.
            //And the nodes CAN move.
            var spareNodes = new QuickQueue<int>(BufferPools<int>.Thread, 8);
            var refinementFlags = new int[leafCount * 2 - 1];
            for (int i = 0; i < nodeCount; ++i)
            {
                refinementFlags[i] = 0;
            }
            for (int i = 0; i < leafCount; ++i)
            {
                TryToBottomUpSweepRefine(refinementFlags, leaves[i].NodeIndex, ref spareNodes);
                //Validate();
            }
            //Console.WriteLine($"root children: {nodes->ChildCount}");
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }

    }
}
