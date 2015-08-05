using Toolbox = BEPUutilities.Toolbox;
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
        unsafe internal struct BinnedSubtrees
        {
            public BoundingBox* BoundingBoxes;
            public int* LeafCounts;
            public int* IndexMap;
            public Vector3* Centroids;
        }

        unsafe void FindPartitionBinned(ref BinnedSubtrees subtrees, int start, int count, ref BoundingBox boundingBox,
               out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {

            //Initialize the per-axis candidate maps.
            var localIndexMap = subtrees.IndexMap + start;
            BoundingBox centroidBoundingBox;
            centroidBoundingBox.Min = subtrees.Centroids[localIndexMap[0]];
            centroidBoundingBox.Max = centroidBoundingBox.Min;
            for (int i = 1; i < count; ++i)
            {
                var centroid = subtrees.Centroids + localIndexMap[i];
                centroidBoundingBox.Min = Vector3.Min(*centroid, centroidBoundingBox.Min);
                centroidBoundingBox.Max = Vector3.Max(*centroid, centroidBoundingBox.Max);
            }

            //Bin along all three axes simultaneously.
            var nullBoundingBox = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            var span = centroidBoundingBox.Max - centroidBoundingBox.Min;
            if (span.X < Toolbox.Epsilon && span.Y < Toolbox.Epsilon && span.Z < Toolbox.Epsilon)
            {
                //All axes are degenerate.
                //This is the one situation in which we can end up with all objects in the same bin.
                //To stop this, just short circuit.
                splitIndex = count / 2;
                a = nullBoundingBox;
                b = nullBoundingBox;
                leafCountA = 0;
                leafCountB = 0;
                for (int i = 0; i < splitIndex; ++i)
                {
                    BoundingBox.Merge(ref a, ref subtrees.BoundingBoxes[localIndexMap[i]], out a);
                    leafCountA += subtrees.LeafCounts[localIndexMap[i]];
                }
                for (int i = splitIndex; i < count; ++i)
                {
                    BoundingBox.Merge(ref b, ref subtrees.BoundingBoxes[localIndexMap[i]], out b);
                    leafCountB += subtrees.LeafCounts[localIndexMap[i]];
                }
                splitIndex += start;
                return;
            }


            const int maximumBinCount = 64;
            //There is no real value in having tons of bins when there are very few children.
            //At low counts, many of them even end up empty.
            //You can get huge speed boosts by simply dropping the bin count adaptively.
            var binCount = (int)Math.Min(maximumBinCount, Math.Max(count * .25f, 2));

            //Take into account zero-width cases.
            //This will result in degenerate axes all being dumped into the first bin.
            var inverseBinSize = new Vector3(
                span.X > 1e-7f ? binCount / span.X : 0,
                span.Y > 1e-7f ? binCount / span.Y : 0,
                span.Z > 1e-7f ? binCount / span.Z : 0);

            //If the span along an axis is too small, just ignore it.
            var maximumBinIndex = new Vector3(binCount - 1);

            //These subtreeBinIndices are potentially very wide. Could do better by pulling from a pool until roslyn stops emitting localsinit for all functions.
            var subtreeBinIndicesX = stackalloc int[count];
            var subtreeBinIndicesY = stackalloc int[count];
            var subtreeBinIndicesZ = stackalloc int[count];
            var binBoundingBoxesX = stackalloc BoundingBox[binCount];
            var binBoundingBoxesY = stackalloc BoundingBox[binCount];
            var binBoundingBoxesZ = stackalloc BoundingBox[binCount];
            var binLeafCountsX = stackalloc int[binCount];
            var binLeafCountsY = stackalloc int[binCount];
            var binLeafCountsZ = stackalloc int[binCount];
            var binSubtreeCountsX = stackalloc int[binCount];
            var binSubtreeCountsY = stackalloc int[binCount];
            var binSubtreeCountsZ = stackalloc int[binCount];

            for (int i = 0; i < binCount; ++i)
            {
                binBoundingBoxesX[i] = nullBoundingBox;
                binBoundingBoxesY[i] = nullBoundingBox;
                binBoundingBoxesZ[i] = nullBoundingBox;

                binLeafCountsX[i] = 0;
                binLeafCountsY[i] = 0;
                binLeafCountsZ[i] = 0;
            }

            //Allocate subtrees to bins for all axes simultaneously.
            for (int i = 0; i < count; ++i)
            {
                var subtreeIndex = localIndexMap[i];
                var binIndices = Vector3.Min((subtrees.Centroids[subtreeIndex] - centroidBoundingBox.Min) * inverseBinSize, maximumBinIndex);
                var x = (int)binIndices.X;
                var y = (int)binIndices.Y;
                var z = (int)binIndices.Z;

                var subtreeBoundingBox = subtrees.BoundingBoxes + subtreeIndex;
                var leafCount = subtrees.LeafCounts + subtreeIndex;
                BoundingBox.Merge(ref binBoundingBoxesX[x], ref *subtreeBoundingBox, out binBoundingBoxesX[x]);
                BoundingBox.Merge(ref binBoundingBoxesY[y], ref *subtreeBoundingBox, out binBoundingBoxesY[y]);
                BoundingBox.Merge(ref binBoundingBoxesZ[z], ref *subtreeBoundingBox, out binBoundingBoxesZ[z]);
                binLeafCountsX[x] += *leafCount;
                binLeafCountsY[y] += *leafCount;
                binLeafCountsZ[z] += *leafCount;
                ++binSubtreeCountsX[x];
                ++binSubtreeCountsY[y];
                ++binSubtreeCountsZ[z];

                subtreeBinIndicesX[i] = x;
                subtreeBinIndicesY[i] = y;
                subtreeBinIndicesZ[i] = z;
            }

            //Determine split axes for all axes simultaneously.
            //Sweep from low to high.
            var lastIndex = binCount - 1;
            var aLeafCountsX = stackalloc int[lastIndex];
            var aLeafCountsY = stackalloc int[lastIndex];
            var aLeafCountsZ = stackalloc int[lastIndex];
            var aMergedX = stackalloc BoundingBox[lastIndex];
            var aMergedY = stackalloc BoundingBox[lastIndex];
            var aMergedZ = stackalloc BoundingBox[lastIndex];

            aLeafCountsX[0] = binLeafCountsX[0];
            aLeafCountsY[0] = binLeafCountsY[0];
            aLeafCountsZ[0] = binLeafCountsZ[0];
            aMergedX[0] = binBoundingBoxesX[0];
            aMergedY[0] = binBoundingBoxesY[0];
            aMergedZ[0] = binBoundingBoxesZ[0];
            for (int i = 1; i < lastIndex; ++i)
            {
                var previousIndex = i - 1;
                aLeafCountsX[i] = binLeafCountsX[i] + aLeafCountsX[previousIndex];
                aLeafCountsY[i] = binLeafCountsY[i] + aLeafCountsY[previousIndex];
                aLeafCountsZ[i] = binLeafCountsZ[i] + aLeafCountsZ[previousIndex];
                BoundingBox.Merge(ref aMergedX[previousIndex], ref binBoundingBoxesX[i], out aMergedX[i]);
                BoundingBox.Merge(ref aMergedY[previousIndex], ref binBoundingBoxesY[i], out aMergedY[i]);
                BoundingBox.Merge(ref aMergedZ[previousIndex], ref binBoundingBoxesZ[i], out aMergedZ[i]);
            }

            //Sweep from high to low.
            BoundingBox bMergedX = nullBoundingBox;
            BoundingBox bMergedY = nullBoundingBox;
            BoundingBox bMergedZ = nullBoundingBox;
            int bLeafCountX = 0;
            int bLeafCountY = 0;
            int bLeafCountZ = 0;

            int bestAxis = 0;
            float cost = float.MaxValue;
            var binSplitIndex = 0;
            a = nullBoundingBox;
            b = nullBoundingBox;
            leafCountA = 0;
            leafCountB = 0;

            int totalLeafCount = 0;
            for (int i = 0; i < count; ++i)
            {
                totalLeafCount += subtrees.LeafCounts[localIndexMap[i]];
            }

            for (int i = lastIndex; i >= 1; --i)
            {
                int aIndex = i - 1;
                BoundingBox.Merge(ref bMergedX, ref binBoundingBoxesX[i], out bMergedX);
                BoundingBox.Merge(ref bMergedY, ref binBoundingBoxesY[i], out bMergedY);
                BoundingBox.Merge(ref bMergedZ, ref binBoundingBoxesZ[i], out bMergedZ);
                bLeafCountX += binLeafCountsX[i];
                bLeafCountY += binLeafCountsY[i];
                bLeafCountZ += binLeafCountsZ[i];

                var metricAX = ComputeBoundsMetric(ref aMergedX[aIndex]);
                var metricAY = ComputeBoundsMetric(ref aMergedY[aIndex]);
                var metricAZ = ComputeBoundsMetric(ref aMergedZ[aIndex]);
                var metricBX = ComputeBoundsMetric(ref bMergedX);
                var metricBY = ComputeBoundsMetric(ref bMergedY);
                var metricBZ = ComputeBoundsMetric(ref bMergedZ);

                //It's possible for a lot of bins in a row to be unpopulated. In that event, you'll get a metric of -infinity. Avoid letting that propagate.
                float costCandidateX, costCandidateY, costCandidateZ;
                if (metricAX > 0 && metricBX > 0)
                    costCandidateX = aLeafCountsX[aIndex] * metricAX + bLeafCountX * metricBX;
                else
                    costCandidateX = float.MaxValue;
                if (metricAY > 0 && metricBY > 0)
                    costCandidateY = aLeafCountsY[aIndex] * metricAY + bLeafCountY * metricBY;
                else
                    costCandidateY = float.MaxValue;
                if (metricAZ > 0 && metricBZ > 0)
                    costCandidateZ = aLeafCountsZ[aIndex] * metricAZ + bLeafCountZ * metricBZ;
                else
                    costCandidateZ = float.MaxValue;
                if (costCandidateX < costCandidateY && costCandidateX < costCandidateZ)
                {
                    if (costCandidateX < cost)
                    {
                        bestAxis = 0;
                        cost = costCandidateX;
                        binSplitIndex = i;
                        a = aMergedX[aIndex];
                        b = bMergedX;
                        leafCountA = aLeafCountsX[aIndex];
                        leafCountB = bLeafCountX;
                    }
                }
                else if (costCandidateY < costCandidateZ)
                {
                    if (costCandidateY < cost)
                    {
                        bestAxis = 1;
                        cost = costCandidateY;
                        binSplitIndex = i;
                        a = aMergedY[aIndex];
                        b = bMergedY;
                        leafCountA = aLeafCountsY[aIndex];
                        leafCountB = bLeafCountY;
                    }
                }
                else
                {
                    if (costCandidateZ < cost)
                    {
                        bestAxis = 2;
                        cost = costCandidateZ;
                        binSplitIndex = i;
                        a = aMergedZ[aIndex];
                        b = bMergedZ;
                        leafCountA = aLeafCountsZ[aIndex];
                        leafCountB = bLeafCountZ;
                    }
                }

            }


            int* bestBinSubtreeCounts;
            int* bestSubtreeBinIndices;
            switch (bestAxis)
            {
                case 0:
                    bestBinSubtreeCounts = binSubtreeCountsX;
                    bestSubtreeBinIndices = subtreeBinIndicesX;
                    break;
                case 1:
                    bestBinSubtreeCounts = binSubtreeCountsY;
                    bestSubtreeBinIndices = subtreeBinIndicesY;
                    break;
                default:
                    bestBinSubtreeCounts = binSubtreeCountsZ;
                    bestSubtreeBinIndices = subtreeBinIndicesZ;
                    break;
            }
            //Rebuild the index map.
            var binStartIndices = stackalloc int[binCount];
            var binSubtreeCountsSecondPass = stackalloc int[binCount];
            var tempIndexMap = stackalloc int[count];

            binStartIndices[0] = 0;
            binSubtreeCountsSecondPass[0] = 0;

            for (int i = 1; i < binCount; ++i)
            {
                binStartIndices[i] = binStartIndices[i - 1] + bestBinSubtreeCounts[i - 1];
                binSubtreeCountsSecondPass[i] = 0;
            }

            for (int i = 0; i < count; ++i)
            {
                tempIndexMap[binStartIndices[bestSubtreeBinIndices[i]] + binSubtreeCountsSecondPass[bestSubtreeBinIndices[i]]++] = localIndexMap[i];
            }

            //Update the real index map.
            for (int i = 0; i < count; ++i)
            {
                localIndexMap[i] = tempIndexMap[i];
            }

            //Transform the split index into object indices.
            splitIndex = binStartIndices[binSplitIndex] + start;

        }



        unsafe void SplitSubtreesIntoChildrenBinned(int depthRemaining, ref BinnedSubtrees subtrees,
            int start, int count, ref BoundingBox boundingBox,
            Node* stagingNodes, int stagingNodeIndex, ref int stagingNodesCount, out float childrenTreeletsCost)
        {
            if (count > 1)
            {

                BoundingBox a, b;
                int leafCountA, leafCountB;
                int splitIndex;
                FindPartitionBinned(ref subtrees, start, count, ref boundingBox, out splitIndex, out a, out b, out leafCountA, out leafCountB);


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
            ref BinnedSubtrees subtrees, int start, int count,
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
            var centroids = stackalloc Vector3[subtreeReferences.Count];
            BinnedSubtrees subtrees;
            subtrees.BoundingBoxes = boundingBoxes;
            subtrees.LeafCounts = leafCounts;
            subtrees.IndexMap = indexMap;
            subtrees.Centroids = centroids;

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
                    centroids[i] = boundingBox->Min + boundingBox->Max;
                    leafCounts[i] = (&parentNode->LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node.
                    var leaf = leaves + Encode(subtreeReferences.Elements[i]);
                    *boundingBox = (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                    centroids[i] = boundingBox->Min + boundingBox->Max;
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

            if (true) // newTreeletCost < originalTreeletCost)
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
