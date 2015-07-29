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
        struct SweepSubtree
        {
            public BoundingBox BoundingBox;
            public int Index;
            public int LeafCount;
        }

        unsafe void ComputeBoundingBox(SweepSubtree* subtrees, int subtreeStart, int subtreeCount, out BoundingBox boundingBox)
        {
            Debug.Assert(subtreeCount > 0);
            Debug.Assert(subtreeStart >= 0);
            subtrees += subtreeStart;

            boundingBox = subtrees->BoundingBox;
            for (int i = 1; i < subtreeCount; ++i)
            {
                BoundingBox.Merge(ref subtrees[i].BoundingBox, ref boundingBox, out boundingBox);
            }
        }

        unsafe void CentroidSort(SweepSubtree* subtrees, int subtreeStart, int subtreeCount, ref BoundingBox boundingBox)
        {
            //Compute dominant axis based on the bounding box.
            //Sort along that axis. This uses an in-place insertion sort for simplicity and to take advantage of the partially sorted data.
            var offset = boundingBox.Max - boundingBox.Min;
            int axisIndex;
            if (offset.X >= offset.Y && offset.X >= offset.Z)
            {
                //X is dominant.
                axisIndex = 0;
            }
            else if (offset.Y >= offset.Z)
            {
                //Y is dominant.
                axisIndex = 1;
            }
            else
            {
                //Z is dominant.
                axisIndex = 2;
            }
            //A variety of potential optimizations exist here.
            //Don't reallocate centroids (because JIT is forced to zero by roslyn), do swaps better, etc.
            //May also want to try not accessing components via pointer- may be faster to do directly, not much extra work.
            var centroids = stackalloc float[subtreeCount];
            subtrees += subtreeStart;
            for (int i = 0; i < subtreeCount; ++i)
            {
                centroids[i] = (&subtrees[i].BoundingBox.Min.X)[axisIndex] + (&subtrees[i].BoundingBox.Max.X)[axisIndex];
            }
            for (int i = 1; i < subtreeCount; ++i)
            {
                var index = i;
                var previousIndex = index - 1;
                while (centroids[index] < centroids[previousIndex])
                {
                    var tempCentroid = centroids[index];
                    var tempSubtree = subtrees[index];
                    centroids[index] = centroids[previousIndex];
                    subtrees[index] = subtrees[previousIndex];
                    centroids[previousIndex] = tempCentroid;
                    subtrees[previousIndex] = tempSubtree;

                    index = previousIndex;
                    --previousIndex;
                }
            }
        }

        unsafe int GetSplitIndex(SweepSubtree* subtrees, int subtreeStart, int subtreeCount, out BoundingBox a, out BoundingBox b, out int aLeafCount, out int bLeafCount)
        {
            //Precompute the bounding boxes to avoid redundant work.
            //TODO: Could avoid reallocating this array over and over again.
            //Build the bMerged lists by iterating backwards through the sorted leaves. Only do length - 1, since 0 < split < length - 1.
            //We want to just index bMerged directly with the candidate splitIndex to get the associated bounding boxes.
            //So, bMerged[0] should contain all bounding boxes from leaves[start + 1] to leaves[start + length - 1].
            //bMerged[bMerged.Length - 1] is just leaves[start + length - 1]'s bounding box.
            var bMergedLength = subtreeCount - 1;
            var bMerged = stackalloc BoundingBox[bMergedLength];
            var bLeafCounts = stackalloc int[bMergedLength];
            var lastIndex = subtreeStart + subtreeCount - 1;

            bMerged[bMergedLength - 1] = subtrees[lastIndex].BoundingBox;
            bLeafCounts[bMergedLength - 1] = subtrees[lastIndex].LeafCount;
            for (int i = bMergedLength - 2; i >= 0; --i)
            {
                var subtreeIndex = subtreeStart + 1 + i;
                bLeafCounts[i] = bLeafCounts[i + 1] + subtrees[subtreeIndex].LeafCount;
                BoundingBox.Merge(ref subtrees[subtreeIndex].BoundingBox, ref bMerged[i + 1], out bMerged[i]);

            }

            int lowestIndex = -1;
            float lowestCost = float.MaxValue;
            BoundingBox merged = new BoundingBox
            {
                Min = new Vector3(float.MaxValue),
                Max = new Vector3(-float.MaxValue)
            };
            a = b = merged;
            aLeafCount = bLeafCount = 0;
            int leafCount = 0;
            for (int i = 0; i < subtreeCount - 1; ++i)
            {
                leafCount += subtrees[i].LeafCount;
                BoundingBox.Merge(ref merged, ref subtrees[subtreeStart + i].BoundingBox, out merged);
                var candidateCost = leafCount * ComputeBoundsMetric(ref merged) + bLeafCounts[i] * ComputeBoundsMetric(ref bMerged[i]);
                if (candidateCost < lowestCost)
                {
                    lowestCost = candidateCost;
                    lowestIndex = i;
                    a = merged;
                    b = bMerged[i];
                    aLeafCount = LeafCount;
                    bLeafCount = bLeafCounts[i];
                }
            }
            return subtreeStart + lowestIndex + 1;
        }


        unsafe void SplitSubtreesIntoChildren(int depthRemaining, SweepSubtree* subtrees, int subtreeStart, int subtreeCount, ref BoundingBox boundingBox,
            Node* stagingNodes, int stagingNodeIndex, ref int stagingNodesCount, out float childrenTreeletsCost)
        {
            if (subtreeCount > 1)
            {
                CentroidSort(subtrees, subtreeStart, subtreeCount, ref boundingBox);

                BoundingBox a, b;
                int leafCountA, leafCountB;
                var splitIndex = GetSplitIndex(subtrees, subtreeStart, subtreeCount, out a, out b, out leafCountA, out leafCountB);
                float costA, costB;
                if (depthRemaining > 0)
                {
                    --depthRemaining;
                    SplitSubtreesIntoChildren(depthRemaining, subtrees, subtreeStart, splitIndex - subtreeStart, ref a, stagingNodes, stagingNodeIndex, ref stagingNodesCount, out costA);
                    SplitSubtreesIntoChildren(depthRemaining, subtrees, splitIndex, subtreeStart + subtreeCount - splitIndex, ref b, stagingNodes, stagingNodeIndex, ref stagingNodesCount, out costB);
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
                    var stagingLeafCounts = &stagingNodes->LeafCountA;

                    stagingBounds[childIndexA] = a;
                    stagingBounds[childIndexB] = b;

                    stagingLeafCounts[childIndexA] = leafCountA;
                    stagingLeafCounts[childIndexB] = leafCountB;

                    int subtreeCountA = splitIndex - subtreeStart;
                    int subtreeCountB = subtreeStart + subtreeCount - splitIndex;
                    if (subtreeCountA > 1)
                    {
                        stagingChildren[childIndexA] = CreateStagingNode(stagingNodeIndex, childIndexA, ref a, subtrees, subtreeStart, subtreeCountA,
                            stagingNodes, ref stagingNodesCount, out costA);
                    }
                    else
                    {
                        Debug.Assert(subtreeCountA == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexA] = Encode(subtreeStart);
                        costA = 0;
                    }
                    if (subtreeCountB > 1)
                    {
                        stagingChildren[childIndexB] = CreateStagingNode(stagingNodeIndex, childIndexB, ref b, subtrees, splitIndex, subtreeCountB,
                            stagingNodes, ref stagingNodesCount, out costB);
                    }
                    else
                    {
                        Debug.Assert(subtreeCountB == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexB] = Encode(splitIndex);
                        costB = 0;
                    }
                }
                childrenTreeletsCost = costA + costB;
            }
            else
            {
                Debug.Assert(subtreeCount == 1);
                //Only one subtree. Just stick it directly into the node.
                var childIndex = stagingNodes[stagingNodeIndex].ChildCount++;
                Debug.Assert(stagingNodes[stagingNodeIndex].ChildCount <= ChildrenCapacity);
                (&stagingNodes[stagingNodeIndex].A)[childIndex] = subtrees[subtreeStart].BoundingBox;
                (&stagingNodes[stagingNodeIndex].ChildA)[childIndex] = Encode(subtreeStart);
                (&stagingNodes[stagingNodeIndex].LeafCountA)[childIndex] = subtrees[subtreeStart].LeafCount;
                //Subtrees cannot contribute to change in cost.
                childrenTreeletsCost = 0;
            }
        }

        unsafe int CreateStagingNode(int parentIndex, int indexInParent, ref BoundingBox boundingBox,
            SweepSubtree* subtrees, int subtreeStart, int subtreeCount,
            Node* stagingNodes, ref int stagingNodeCount, out float childTreeletsCost)
        {
            var stagingNodeIndex = stagingNodeCount++;
            var stagingNode = stagingNodes + stagingNodeIndex;

            if (subtreeCount < ChildrenCapacity)
            {
                //No need to do any sorting. This node can fit every remaining subtree.
                subtrees += subtreeStart;
                stagingNode->ChildCount = subtreeCount;
                var stagingNodeBounds = &stagingNode->A;
                var stagingNodeChildren = &stagingNode->ChildA;
                var leafCounts = &stagingNode->LeafCountA;
                for (int i = 0; i < subtreeCount; ++i)
                {
                    stagingNodeBounds[i] = subtrees[i].BoundingBox;
                    leafCounts[i] = subtrees[i].LeafCount;
                    stagingNodeChildren[i] = subtrees[i].Index;
                }
                //Because subtrees do not change in size, they cannot change the cost.
                childTreeletsCost = 0;
                return stagingNodeIndex;
            }

            const int recursionDepth = ChildrenCapacity == 32 ? 5 : ChildrenCapacity == 16 ? 4 : ChildrenCapacity == 8 ? 3 : ChildrenCapacity == 4 ? 2 : 1;


            SplitSubtreesIntoChildren(recursionDepth, subtrees, subtreeStart, subtreeCount, ref boundingBox, stagingNodes, stagingNodeIndex, ref stagingNodeCount, out childTreeletsCost);


            return stagingNodeIndex;

        }

        public unsafe void SweepRefine(int nodeIndex, ref QuickList<int> internalNodes, out bool nodesInvalidated)
        {
            const int maximumSubtrees = 4096;
            var subtrees = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            int internalNodeStartIndex = internalNodes.Count;
            float originalTreeletCost;
            CollectSubtrees(0, maximumSubtrees, ref subtrees, ref internalNodes, out originalTreeletCost);

            //Gather necessary information from nodes. (TODO: This could be more efficiently gathered up front... collectsubtrees already touched most of this data!)
            var sweepSubtrees = stackalloc SweepSubtree[subtrees.Count];
            for (int i = 0; i < subtrees.Count; ++i)
            {
                var subtree = sweepSubtrees + i;
                subtree->Index = subtrees.Elements[i];
                if (subtree->Index >= 0)
                {
                    //It's an internal node.
                    var subtreeNode = nodes + subtree->Index;
                    var parentNode = nodes + subtreeNode->Parent;
                    subtree->BoundingBox = (&parentNode->A)[subtreeNode->IndexInParent];
                    subtree->LeafCount = (&parentNode->LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node.
                    subtree->LeafCount = 1;
                    var leaf = leaves + Encode(subtree->Index);
                    subtree->BoundingBox = (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                }
            }

            var node = nodes + nodeIndex;
            int parent = node->Parent;
            int indexInParent = node->IndexInParent;

            //Now perform a top-down sweep build.
            //TODO: this staging creation section is really the only part that is sweep-specific. The rest is common to any other kind of subtree-collection based refinement. 
            //If you end up making others, keep this in mind.
            int stagingNodeCount = 0;
            int stagingNodeCapacity = subtrees.Count - 1;
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
            CreateStagingNode(parent, indexInParent, ref treeletBoundingBox, sweepSubtrees, 0, subtrees.Count, stagingNodes, ref stagingNodeCount, out newTreeletCost);

            if (newTreeletCost < originalTreeletCost)
            {
                //Reify the nodes.
                ReifyStagingNode(parent, indexInParent, stagingNodes, 0, stagingNodeCapacity, ref subtrees, ref internalNodes, out nodesInvalidated);
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

            subtrees.Dispose();
        }


        private unsafe void TopDownSweepRefine(int nodeIndex, ref QuickList<int> spareNodes)
        {
            bool nodesInvalidated;
            AgglomerativeRefine(nodeIndex, ref spareNodes, out nodesInvalidated);
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
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
            TopDownSweepRefine(0, ref spareNodes);
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }
    }
}
