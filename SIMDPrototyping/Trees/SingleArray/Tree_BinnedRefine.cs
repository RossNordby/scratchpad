using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {
        unsafe void FindPartitionForAxisBinned(BoundingBox* boundingBoxes, int* leafCounts, float* centroids, int* originalIndexMap, int* indexMap, int subtreeCount,
            out int splitIndex, out float cost, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            Debug.Assert(subtreeCount > 1);

            const int binCount = 16;
            const float inverseBinCount = 1f / binCount;

            //TODO: Try out AOS centroids again or figure out a clever SOA approach.
            //Compute centroid bounds. We use these instead of the bounding box because
            //the whole bounding can end up distributing ALL subtrees into a single bin.
            //(Consider one subtree with an enormous AABB...)
            //While that's a corner case, it still has an effect on average.
            var centroid = centroids[*originalIndexMap];
            float min = centroid;
            float max = centroid;
            for (int i = 1; i < subtreeCount; ++i)
            {
                centroid = centroids[originalIndexMap[i]];
                if (centroid < min)
                {
                    min = centroid;
                }
                else if (centroid > max)
                {
                    max = centroid;
                }
            }

            var span = max - min;
            var multiplier = span * inverseBinCount;
            //Ever so slightly offset the min to keep things predictable.
            min -= span * 1e-5f;

            var subtreeBinIndices = new int[subtreeCount];
            var binSubtreeCounts = new int[binCount];
            var binSubtreeCountsSecondPass = new int[binCount];
            var binBoundingBoxes = new BoundingBox[binCount];
            var binLeafCounts = new int[binCount];
            //Initialize to default values. (Stackalloc actually does this for the numerical types, but we can't assume that.)
            for (int i = 0; i < binCount; ++i)
            {
                binBoundingBoxes[i] = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
                binLeafCounts[i] = 0;
                binSubtreeCounts[i] = 0;
                binSubtreeCountsSecondPass[i] = 0;
            }

            //Determine which bins each subtree belongs to.
            //TODO: Note that centroid computation and bin index computation can be done in a pretty SIMD fashion (3-wide, at least).
            //Actual depositing part is scalar, but some speedups still might be possible.
            for (int i = 0; i < subtreeCount; ++i)
            {
                var subtreeIndex = originalIndexMap[i];
                var binIndex = (int)((centroids[subtreeIndex] - min) * multiplier);
                BoundingBox.Merge(ref binBoundingBoxes[binIndex], ref boundingBoxes[subtreeIndex], out binBoundingBoxes[binIndex]);
                binLeafCounts[i] += leafCounts[subtreeIndex];
            }

            //Rebuild the index map.
            //Technically, could shave some of this work off until you knew for a fact that this was the winning candidate by cost.
            var binStartIndices = new int[binCount];
            binStartIndices[0] = 0;
            for (int i = 1; i < binCount; ++i)
            {
                binStartIndices[i] = binStartIndices[i - 1] + subtreeBinIndices[i - 1];
            }
            for (int i = 0; i < subtreeCount; ++i)
            {
                indexMap[i] = originalIndexMap[binStartIndices[subtreeBinIndices[i]] + binSubtreeCountsSecondPass[i]++];
            }

            //Determine the split index.
            var lastIndex = binCount - 1;
            var aLeafCounts = new int[lastIndex];
            var aMerged = new BoundingBox[lastIndex];

            aLeafCounts[0] = binLeafCounts[0];
            aMerged[0] = binBoundingBoxes[0];
            for (int i = 1; i < lastIndex; ++i)
            {
                aLeafCounts[i] = binLeafCounts[i] + aLeafCounts[i - 1];
                BoundingBox.Merge(ref aMerged[i - 1], ref binBoundingBoxes[i], out aMerged[i]);
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
                BoundingBox.Merge(ref bMerged, ref binBoundingBoxes[i], out bMerged);
                bLeafCount += binLeafCounts[i];

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
            //The split index was in terms of bins. Turn it into a subtree index.
            splitIndex = binStartIndices[splitIndex];


        }
        unsafe void FindPartitionBinned(ref Subtrees subtrees, int start, int count,
               out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            //A variety of potential microoptimizations exist here.
            //Don't reallocate centroids (because JIT is forced to zero by roslyn), do swaps better, etc.
            var centroids = stackalloc Vector3[count];
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
            FindPartitionForAxisBinned(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsX, localIndexMap, indexMapX, count, out xSplitIndex, out xCost, out xA, out xB, out xLeafCountA, out xLeafCountB);
            FindPartitionForAxisBinned(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsY, localIndexMap, indexMapY, count, out ySplitIndex, out yCost, out yA, out yB, out yLeafCountA, out yLeafCountB);
            FindPartitionForAxisBinned(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsZ, localIndexMap, indexMapZ, count, out zSplitIndex, out zCost, out zA, out zB, out zLeafCountA, out zLeafCountB);

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



        unsafe void SplitSubtreesIntoChildrenBinned(int depthRemaining, ref Subtrees subtrees,
            int start, int count, ref BoundingBox boundingBox,
            Node* stagingNodes, int stagingNodeIndex, ref int stagingNodesCount, out float childrenTreeletsCost)
        {
            if (count > 1)
            {

                BoundingBox a, b;
                int leafCountA, leafCountB;
                int splitIndex;
                FindPartitionBinned(ref subtrees, start, count, out splitIndex, out a, out b, out leafCountA, out leafCountB);

                float costA, costB;
                if (depthRemaining > 0)
                {
                    --depthRemaining;
                    SplitSubtreesIntoChildrenBinned(depthRemaining, ref subtrees, start, splitIndex - start, ref a, stagingNodes, stagingNodeIndex, ref stagingNodesCount, out costA);
                    SplitSubtreesIntoChildrenBinned(depthRemaining, ref subtrees, splitIndex, start + count - splitIndex, ref b, stagingNodes, stagingNodeIndex, ref stagingNodesCount, out costB);
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
                        stagingChildren[childIndexA] = CreateStagingNodeBinned(stagingNodeIndex, childIndexA, ref a, ref subtrees, start, subtreeCountA,
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
                        stagingChildren[childIndexB] = CreateStagingNodeBinned(stagingNodeIndex, childIndexB, ref b, ref subtrees, splitIndex, subtreeCountB,
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

        unsafe int CreateStagingNodeBinned(int parentIndex, int indexInParent, ref BoundingBox boundingBox,
            ref Subtrees subtrees, int start, int count,
            Node* stagingNodes, ref int stagingNodeCount, out float childTreeletsCost)
        {
            var stagingNodeIndex = stagingNodeCount++;
            var stagingNode = stagingNodes + stagingNodeIndex;

            if (count < ChildrenCapacity)
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


            SplitSubtreesIntoChildrenBinned(recursionDepth, ref subtrees, start, count, ref boundingBox, stagingNodes, stagingNodeIndex, ref stagingNodeCount, out childTreeletsCost);


            return stagingNodeIndex;

        }



        public unsafe void BinnedRefine(int nodeIndex, ref QuickList<int> internalNodes, out bool nodesInvalidated)
        {
            const int maximumSubtrees = 1024;
            var subtreeReferences = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            int internalNodeStartIndex = internalNodes.Count;
            float originalTreeletCost;

            CollectSubtrees(nodeIndex, maximumSubtrees, ref subtreeReferences, ref internalNodes, out originalTreeletCost);


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
            CreateStagingNodeBinned(parent, indexInParent, ref treeletBoundingBox, ref subtrees, 0, subtreeReferences.Count, stagingNodes, ref stagingNodeCount, out newTreeletCost);

            //ValidateStaging(stagingNodes, sweepSubtrees, ref subtreeReferences, parent, indexInParent);

            if (newTreeletCost < originalTreeletCost)
            {
                //Reify the nodes.
                //ValidateLeaves();
                ReifyStagingNode(parent, indexInParent, stagingNodes, 0, stagingNodeCapacity, ref subtreeReferences, ref internalNodes, out nodesInvalidated);
                //ValidateLeaves();
            }
            else
            {
                //The internal nodes collected by the most recent iteration of CollectSubtrees weren't replaced! Get them out of the pool.
                //TODO: Would be nice to do this in a slightly less gross way.
                for (int i = internalNodes.Count - 1; i >= internalNodeStartIndex; --i)
                {
                    internalNodes.FastRemoveAt(i);
                }
                nodesInvalidated = false;
            }

            subtreeReferences.Dispose();

        }





        private unsafe void TopDownBinnedRefine(int nodeIndex, ref QuickList<int> spareNodes)
        {
            bool nodesInvalidated;
            //Validate();
            BinnedRefine(nodeIndex, ref spareNodes, out nodesInvalidated);
            //Validate();
            //The root of the tree is guaranteed to stay in position, so nodeIndex is still valid.

            //Node pointers can be invalidated, so don't hold a reference between executions.
            for (int i = 0; i < nodes[nodeIndex].ChildCount; ++i)
            {
                var child = (&nodes[nodeIndex].ChildA)[i];
                if (child >= 0)
                {
                    TopDownBinnedRefine(child, ref spareNodes);
                }
            }
        }
        public unsafe void TopDownBinnedRefine()
        {
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
            TopDownBinnedRefine(0, ref spareNodes);
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }


        unsafe void TryToBottomUpBinnedRefine(int[] refinementFlags, int nodeIndex, ref QuickList<int> spareInternalNodes)
        {
            if (++refinementFlags[nodeIndex] == nodes[nodeIndex].ChildCount)
            {
                bool nodesInvalidated;
                BinnedRefine(nodeIndex, ref spareInternalNodes, out nodesInvalidated);

                var parent = nodes[nodeIndex].Parent;
                if (parent != -1)
                {
                    TryToBottomUpBinnedRefine(refinementFlags, parent, ref spareInternalNodes);
                }
            }
        }


        /// <summary>
        /// Executes one pass of bottom-up refinement.
        /// </summary>
        public unsafe void BottomUpBinnedRefine()
        {
            //If this works out, should probably choose a more efficient flagging approach.
            //Note the size: it needs to contain all possible internal nodes.
            //TODO: This is actually bugged, because the refinement flags do not update if the nodes move.
            //And the nodes CAN move.
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
            var refinementFlags = new int[leafCount * 2 - 1];
            for (int i = 0; i < nodeCount; ++i)
            {
                refinementFlags[i] = 0;
            }
            for (int i = 0; i < leafCount; ++i)
            {
                TryToBottomUpBinnedRefine(refinementFlags, leaves[i].NodeIndex, ref spareNodes);
                //Validate();
            }
            //Console.WriteLine($"root children: {nodes->ChildCount}");
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }
    }
}
