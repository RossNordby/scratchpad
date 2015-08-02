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

        unsafe void FindPartitionBinned5(ref BinnedSubtrees subtrees, int start, int count,
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
                return;
            }
            else
            {
                //Modify the span to handle degenerate cases.
                //This will result in degenerate axes all being dumped into the first bin.
                if (span.X < 1e-7f)
                    span.X = float.MaxValue;
                if (span.Y < 1e-7f)
                    span.Y = float.MaxValue;
                if (span.Z < 1e-7f)
                    span.Z = float.MaxValue;
            }
            const int binCount = 16;
            var inverseBinSize = new Vector3(binCount) / span;

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

                var boundingBox = subtrees.BoundingBoxes + subtreeIndex;
                var leafCount = subtrees.LeafCounts + subtreeIndex;
                BoundingBox.Merge(ref binBoundingBoxesX[x], ref *boundingBox, out binBoundingBoxesX[x]);
                BoundingBox.Merge(ref binBoundingBoxesY[y], ref *boundingBox, out binBoundingBoxesY[y]);
                BoundingBox.Merge(ref binBoundingBoxesZ[z], ref *boundingBox, out binBoundingBoxesZ[z]);
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
            aLeafCountsX[0] = binLeafCountsZ[0];
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
            splitIndex = 0;
            a = nullBoundingBox;
            b = nullBoundingBox;
            leafCountA = 0;
            leafCountB = 0;

            for (int i = lastIndex; i >= 1; --i)
            {
                int aIndex = i - 1;
                BoundingBox.Merge(ref bMergedX, ref binBoundingBoxesX[i], out bMergedX);
                BoundingBox.Merge(ref bMergedY, ref binBoundingBoxesY[i], out bMergedY);
                BoundingBox.Merge(ref bMergedZ, ref binBoundingBoxesZ[i], out bMergedZ);
                bLeafCountX += binLeafCountsX[i];
                bLeafCountY += binLeafCountsY[i];
                bLeafCountZ += binLeafCountsZ[i];

                var costCandidateX = aLeafCountsX[aIndex] * ComputeBoundsMetric(ref aMergedX[aIndex]) + bLeafCountX * ComputeBoundsMetric(ref bMergedX);
                var costCandidateY = aLeafCountsY[aIndex] * ComputeBoundsMetric(ref aMergedY[aIndex]) + bLeafCountY * ComputeBoundsMetric(ref bMergedY);
                var costCandidateZ = aLeafCountsZ[aIndex] * ComputeBoundsMetric(ref aMergedZ[aIndex]) + bLeafCountZ * ComputeBoundsMetric(ref bMergedZ);

                if (costCandidateX < costCandidateY && costCandidateY < costCandidateZ)
                {
                    if (costCandidateX < cost)
                    {
                        bestAxis = 0;
                        cost = costCandidateX;
                        splitIndex = i;
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
                        splitIndex = i;
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
                        splitIndex = i;
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
                tempIndexMap[i] = localIndexMap[binStartIndices[bestSubtreeBinIndices[i]] + binSubtreeCountsSecondPass[bestSubtreeBinIndices[i]]++];
            }

            //Update the real index map.
            for (int i = 0; i < count; ++i)
            {
                localIndexMap[i] = tempIndexMap[i];
            }
            
        

            //Transform the split index into object indices.
            splitIndex = binStartIndices[splitIndex] + start;



        }

        //unsafe void FindPartitionForAxisBinned(BoundingBox* boundingBoxes, int* leafCounts, float* centroids, int* originalIndexMap, int* indexMap, int subtreeCount,
        //    out int splitIndex, out float cost, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        //{
        //    Debug.Assert(subtreeCount > 1);

        //    const int binCount = 16;


        //    //TODO: Try out AOS centroids again or figure out a clever SOA approach.
        //    //Compute centroid bounds. We use these instead of the bounding box because
        //    //the whole bounding can end up distributing ALL subtrees into a single bin.
        //    //(Consider one subtree with an enormous AABB...)
        //    //While that's a corner case, it still has an effect on average.

        //    var centroid = centroids[*originalIndexMap];
        //    float min = centroid;
        //    float max = centroid;
        //    for (int i = 1; i < subtreeCount; ++i)
        //    {
        //        centroid = centroids[originalIndexMap[i]];
        //        if (centroid < min)
        //        {
        //            min = centroid;
        //        }
        //        else if (centroid > max)
        //        {
        //            max = centroid;
        //        }
        //    }


        //    var span = max - min;
        //    if (span < 1e-6f)
        //    {
        //        //Everything appears to be in the same spot.
        //        //No real good options.
        //        //Just use the existing layout.
        //        for (int i = 0; i < subtreeCount; ++i)
        //        {
        //            indexMap[i] = originalIndexMap[i];
        //        }
        //        //Pick the middle as the split index.
        //        splitIndex = subtreeCount / 2;
        //        //Build the children properties.
        //        a = boundingBoxes[indexMap[0]];
        //        leafCountA = leafCounts[indexMap[0]];
        //        for (int i = 1; i < splitIndex; ++i)
        //        {
        //            BoundingBox.Merge(ref a, ref boundingBoxes[indexMap[i]], out a);
        //            leafCountA += leafCounts[indexMap[i]];
        //        }
        //        b = boundingBoxes[indexMap[splitIndex]];
        //        leafCountB = leafCounts[indexMap[0]];
        //        for (int i = splitIndex + 1; i < subtreeCount; ++i)
        //        {
        //            BoundingBox.Merge(ref b, ref boundingBoxes[indexMap[i]], out b);
        //            leafCountB += leafCounts[indexMap[i]];
        //        }
        //        cost = ComputeBoundsMetric(ref a) * leafCountA + ComputeBoundsMetric(ref b) * leafCountB;
        //        return;

        //    }
        //    var inverseBinSize = binCount / span;
        //    //Ever so slightly offset the min to keep things predictable.
        //    min += span * 1e-5f;

        //    var subtreeBinIndices = stackalloc int[subtreeCount];
        //    var binSubtreeCounts = stackalloc int[binCount];
        //    var binSubtreeCountsSecondPass = stackalloc int[binCount];
        //    var binBoundingBoxes = stackalloc BoundingBox[binCount];
        //    var binLeafCounts = stackalloc int[binCount];
        //    //Initialize to default values. (Stackalloc actually does this for the numerical types, but we can't assume that.)
        //    for (int i = 0; i < binCount; ++i)
        //    {
        //        binBoundingBoxes[i] = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
        //        binLeafCounts[i] = 0;
        //        binSubtreeCounts[i] = 0;
        //        binSubtreeCountsSecondPass[i] = 0;
        //    }

        //    //Determine which bins each subtree belongs to.
        //    //TODO: Note that centroid computation and bin index computation can be done in a pretty SIMD fashion (3-wide, at least).
        //    //Actual depositing part is scalar, but some speedups still might be possible.
        //    for (int i = 0; i < subtreeCount; ++i)
        //    {
        //        var subtreeIndex = originalIndexMap[i];
        //        var binIndex = Math.Min((int)((centroids[subtreeIndex] - min) * inverseBinSize), binCount - 1);
        //        BoundingBox.Merge(ref binBoundingBoxes[binIndex], ref boundingBoxes[subtreeIndex], out binBoundingBoxes[binIndex]);
        //        binLeafCounts[binIndex] += leafCounts[subtreeIndex];
        //        ++binSubtreeCounts[binIndex];
        //        subtreeBinIndices[i] = binIndex;
        //    }

        //    //Rebuild the index map.
        //    //Technically, could shave some of this work off until you knew for a fact that this was the winning candidate by cost.
        //    var binStartIndices = stackalloc int[binCount];
        //    binStartIndices[0] = 0;
        //    for (int i = 1; i < binCount; ++i)
        //    {
        //        binStartIndices[i] = binStartIndices[i - 1] + binSubtreeCounts[i - 1];
        //    }
        //    for (int i = 0; i < subtreeCount; ++i)
        //    {
        //        indexMap[i] = originalIndexMap[binStartIndices[subtreeBinIndices[i]] + binSubtreeCountsSecondPass[subtreeBinIndices[i]]++];
        //    }

        //    //Determine the split index.
        //    var lastIndex = binCount - 1;
        //    var aLeafCounts = stackalloc int[lastIndex];
        //    var aMerged = stackalloc BoundingBox[lastIndex];

        //    aLeafCounts[0] = binLeafCounts[0];
        //    aMerged[0] = binBoundingBoxes[0];
        //    for (int i = 1; i < lastIndex; ++i)
        //    {
        //        aLeafCounts[i] = binLeafCounts[i] + aLeafCounts[i - 1];
        //        BoundingBox.Merge(ref aMerged[i - 1], ref binBoundingBoxes[i], out aMerged[i]);
        //    }

        //    //Sweep from high to low.
        //    BoundingBox bMerged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
        //    cost = float.MaxValue;
        //    splitIndex = 0;
        //    int bLeafCount = 0;
        //    a = bMerged;
        //    b = bMerged;
        //    leafCountA = 0;
        //    leafCountB = 0;
        //    for (int i = lastIndex; i >= 1; --i)
        //    {
        //        int aIndex = i - 1;
        //        BoundingBox.Merge(ref bMerged, ref binBoundingBoxes[i], out bMerged);
        //        bLeafCount += binLeafCounts[i];

        //        var aCost = aLeafCounts[aIndex] * ComputeBoundsMetric(ref aMerged[aIndex]);
        //        var bCost = bLeafCount * ComputeBoundsMetric(ref bMerged);

        //        var totalCost = aCost + bCost;
        //        if (totalCost < cost)
        //        {
        //            cost = totalCost;
        //            splitIndex = i;
        //            a = aMerged[aIndex];
        //            b = bMerged;
        //            leafCountA = aLeafCounts[aIndex];
        //            leafCountB = bLeafCount;
        //        }

        //    }
        //    //The split index was in terms of bins. Turn it into a subtree index.
        //    splitIndex = binStartIndices[splitIndex];


        //}
        //unsafe void FindPartitionBinned(ref Subtrees subtrees, int start, int count,
        //       out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        //{
        //    //A variety of potential microoptimizations exist here.
        //    //Don't reallocate centroids (because JIT is forced to zero by roslyn), do swaps better, etc.
        //    var indexMapX = stackalloc int[count];
        //    var indexMapY = stackalloc int[count];
        //    var indexMapZ = stackalloc int[count];

        //    //Initialize the per-axis candidate maps.
        //    var localIndexMap = subtrees.IndexMap + start;

        //    int xSplitIndex, xLeafCountA, xLeafCountB, ySplitIndex, yLeafCountA, yLeafCountB, zSplitIndex, zLeafCountA, zLeafCountB;
        //    BoundingBox xA, xB, yA, yB, zA, zB;
        //    float xCost, yCost, zCost;
        //    FindPartitionForAxisBinned(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsX, localIndexMap, indexMapX, count, out xSplitIndex, out xCost, out xA, out xB, out xLeafCountA, out xLeafCountB);
        //    FindPartitionForAxisBinned(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsY, localIndexMap, indexMapY, count, out ySplitIndex, out yCost, out yA, out yB, out yLeafCountA, out yLeafCountB);
        //    FindPartitionForAxisBinned(subtrees.BoundingBoxes, subtrees.LeafCounts, subtrees.CentroidsZ, localIndexMap, indexMapZ, count, out zSplitIndex, out zCost, out zA, out zB, out zLeafCountA, out zLeafCountB);

        //    int* bestIndexMap;
        //    if (xCost <= yCost && xCost <= zCost)
        //    {
        //        splitIndex = xSplitIndex;
        //        a = xA;
        //        b = xB;
        //        leafCountA = xLeafCountA;
        //        leafCountB = xLeafCountB;
        //        bestIndexMap = indexMapX;
        //    }
        //    else if (yCost <= zCost)
        //    {
        //        splitIndex = ySplitIndex;
        //        a = yA;
        //        b = yB;
        //        leafCountA = yLeafCountA;
        //        leafCountB = yLeafCountB;
        //        bestIndexMap = indexMapY;
        //    }
        //    else
        //    {
        //        splitIndex = zSplitIndex;
        //        a = zA;
        //        b = zB;
        //        leafCountA = zLeafCountA;
        //        leafCountB = zLeafCountB;
        //        bestIndexMap = indexMapZ;
        //    }
        //    for (int i = 0; i < count; ++i)
        //    {
        //        localIndexMap[i] = bestIndexMap[i];
        //    }

        //    splitIndex += start;


        //}




        unsafe void SplitSubtreesIntoChildrenBinned(int depthRemaining, ref BinnedSubtrees subtrees,
            int start, int count, ref BoundingBox boundingBox,
            Node* stagingNodes, int stagingNodeIndex, ref int stagingNodesCount, out float childrenTreeletsCost)
        {
            if (count > 1)
            {

                BoundingBox a, b;
                int leafCountA, leafCountB;
                int splitIndex;
                FindPartitionBinned5(ref subtrees, start, count, out splitIndex, out a, out b, out leafCountA, out leafCountB);

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
