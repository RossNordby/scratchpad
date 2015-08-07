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
using System.Runtime.InteropServices;

namespace SIMDPrototyping.Trees.SingleArray
{
    public unsafe struct BinnedResources
    {
        public BoundingBox* BoundingBoxes;
        public int* LeafCounts;
        public int* IndexMap;
        public Vector3* Centroids;

        public SubtreeHeapEntry* SubtreeHeapEntries;
        public Node* StagingNodes;

        //The binning process requires a lot of auxiliary memory.
        //Rather than continually reallocating it with stackalloc
        //and incurring its zeroing cost (at least until that's improved),
        //create the resources at the beginning of the refinement and reuse them.

        //Subtree related reusable resources.
        public int* SubtreeBinIndicesX;
        public int* SubtreeBinIndicesY;
        public int* SubtreeBinIndicesZ;
        public int* TempIndexMap;

        public int* ALeafCountsX;
        public int* ALeafCountsY;
        public int* ALeafCountsZ;
        public BoundingBox* AMergedX;
        public BoundingBox* AMergedY;
        public BoundingBox* AMergedZ;

        //Bin-space reusable resources.
        public BoundingBox* BinBoundingBoxesX;
        public BoundingBox* BinBoundingBoxesY;
        public BoundingBox* BinBoundingBoxesZ;
        public int* BinLeafCountsX;
        public int* BinLeafCountsY;
        public int* BinLeafCountsZ;
        public int* BinSubtreeCountsX;
        public int* BinSubtreeCountsY;
        public int* BinSubtreeCountsZ;

        public int* BinStartIndices;
        public int* BinSubtreeCountsSecondPass;


    }

    public unsafe
#if DEBUG
        class
#else
        struct
#endif
        MemoryRegion : IDisposable

    {

        GCHandle handle;
        public readonly int Capacity;
        public readonly byte* Memory;

        int memoryAllocated;
        public int BytesAllocated
        {
            get
            {
                return memoryAllocated;
            }
        }


        private static int GetAlignmentOffset(byte* pointer)
        {
            var intPtr = (IntPtr)pointer;
            if (IntPtr.Size == 8)
            {
                var original = intPtr.ToInt64();
                var aligned = (original + 15) & ~0xFL;
                return (int)(aligned - original);
            }
            else
            {
                var original = intPtr.ToInt32();
                var aligned = (original + 15) & ~0xF;
                return aligned - original;
            }
        }


        public MemoryRegion(int[] backingMemory)
        {
            Capacity = backingMemory.Length * sizeof(int);
            memoryAllocated = 0;
            handle = GCHandle.Alloc(backingMemory, GCHandleType.Pinned);
            Memory = (byte*)handle.AddrOfPinnedObject();

            var offset = GetAlignmentOffset(Memory);
            Memory += offset;
            Capacity -= offset;
        }

        public unsafe byte* Allocate(int byteCount)
        {
            var newSize = memoryAllocated + (byteCount + 15) & ~0xF;
            if (newSize > Capacity)
                throw new ArgumentException($"The region can only hold {Capacity - memoryAllocated} additional bytes; {byteCount} will not fit.");
            var toReturn = Memory + memoryAllocated;
            memoryAllocated = newSize;

            return toReturn;

        }

        public void Clear()
        {
            memoryAllocated = 0;
        }

        public void Dispose()
        {
            if (handle.IsAllocated)
            {
                handle.Free();
            }
        }

#if DEBUG
        ~MemoryRegion()
        {
            Debug.Assert(!handle.IsAllocated, "Dispose must be called before the memory region falls out of scope.");
        }
#endif
    }




    partial class Tree
    {
        const int MaximumBinCount = 64;


        public static unsafe void CreateBinnedResources(BufferPool<int> bufferPool, int maximumSubtreeCount, out MemoryRegion region, out BinnedResources resources)
        {
            int nodeCount = maximumSubtreeCount - 1;
            //Note alignment. Probably won't provide any actual benefit- if the CLR doesn't provide aligned memory by default,
            //it's highly unlikely that anything is built to expect or benefit from aligned memory (at the software level, anyway).
            //(And I don't think the vector types are aligned.)
            int bytesRequired =
                16 * (3 + 3 + 1) + sizeof(BoundingBox) * (maximumSubtreeCount + 3 * nodeCount + 3 * MaximumBinCount) +
                16 * (6 + 3 + 8) + sizeof(int) * (maximumSubtreeCount * 6 + nodeCount * 3 + MaximumBinCount * 8) +
                16 * (1) + sizeof(Vector3) * maximumSubtreeCount +
                16 * (1) + sizeof(SubtreeHeapEntry) * maximumSubtreeCount +
                16 * (1) + sizeof(Node) * nodeCount;

            //Divide by 4 due to int.
            //We're using int buffers because they are the most commonly requested resource type, so the resource stands a higher chance of being reused.
            int poolIndex = BufferPool<int>.GetPoolIndex(bytesRequired / 4);

            var buffer = bufferPool.TakeFromPoolIndex(poolIndex);

            region = new MemoryRegion(buffer);

            resources.BoundingBoxes = (BoundingBox*)region.Allocate(sizeof(BoundingBox) * maximumSubtreeCount);
            resources.LeafCounts = (int*)region.Allocate(sizeof(int) * maximumSubtreeCount);
            resources.IndexMap = (int*)region.Allocate(sizeof(int) * maximumSubtreeCount);
            resources.Centroids = (Vector3*)region.Allocate(sizeof(Vector3) * maximumSubtreeCount);
            resources.SubtreeHeapEntries = (SubtreeHeapEntry*)region.Allocate(sizeof(SubtreeHeapEntry) * maximumSubtreeCount);
            resources.StagingNodes = (Node*)region.Allocate(sizeof(Node) * nodeCount);

            resources.SubtreeBinIndicesX = (int*)region.Allocate(sizeof(int) * maximumSubtreeCount);
            resources.SubtreeBinIndicesY = (int*)region.Allocate(sizeof(int) * maximumSubtreeCount);
            resources.SubtreeBinIndicesZ = (int*)region.Allocate(sizeof(int) * maximumSubtreeCount);
            resources.TempIndexMap = (int*)region.Allocate(sizeof(int) * maximumSubtreeCount);

            resources.ALeafCountsX = (int*)region.Allocate(sizeof(int) * nodeCount);
            resources.ALeafCountsY = (int*)region.Allocate(sizeof(int) * nodeCount);
            resources.ALeafCountsZ = (int*)region.Allocate(sizeof(int) * nodeCount);
            resources.AMergedX = (BoundingBox*)region.Allocate(sizeof(BoundingBox) * nodeCount);
            resources.AMergedY = (BoundingBox*)region.Allocate(sizeof(BoundingBox) * nodeCount);
            resources.AMergedZ = (BoundingBox*)region.Allocate(sizeof(BoundingBox) * nodeCount);


            resources.BinBoundingBoxesX = (BoundingBox*)region.Allocate(sizeof(BoundingBox) * MaximumBinCount);
            resources.BinBoundingBoxesY = (BoundingBox*)region.Allocate(sizeof(BoundingBox) * MaximumBinCount);
            resources.BinBoundingBoxesZ = (BoundingBox*)region.Allocate(sizeof(BoundingBox) * MaximumBinCount);
            resources.BinLeafCountsX = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
            resources.BinLeafCountsY = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
            resources.BinLeafCountsZ = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsX = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsY = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsZ = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
            resources.BinStartIndices = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
            resources.BinSubtreeCountsSecondPass = (int*)region.Allocate(sizeof(int) * MaximumBinCount);
        }


        unsafe void FindPartitionBinned(ref BinnedResources resources, int start, int count, ref BoundingBox boundingBox,
               out int splitIndex, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {

            //Initialize the per-axis candidate maps.
            var localIndexMap = resources.IndexMap + start;
            BoundingBox centroidBoundingBox;
            centroidBoundingBox.Min = resources.Centroids[localIndexMap[0]];
            centroidBoundingBox.Max = centroidBoundingBox.Min;
            for (int i = 1; i < count; ++i)
            {
                var centroid = resources.Centroids + localIndexMap[i];
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
                    BoundingBox.Merge(ref a, ref resources.BoundingBoxes[localIndexMap[i]], out a);
                    leafCountA += resources.LeafCounts[localIndexMap[i]];
                }
                for (int i = splitIndex; i < count; ++i)
                {
                    BoundingBox.Merge(ref b, ref resources.BoundingBoxes[localIndexMap[i]], out b);
                    leafCountB += resources.LeafCounts[localIndexMap[i]];
                }
                splitIndex += start;
                return;
            }


            //There is no real value in having tons of bins when there are very few children.
            //At low counts, many of them even end up empty.
            //You can get huge speed boosts by simply dropping the bin count adaptively.
            var binCount = (int)Math.Min(MaximumBinCount, Math.Max(count * .25f, 2));

            //Take into account zero-width cases.
            //This will result in degenerate axes all being dumped into the first bin.
            var inverseBinSize = new Vector3(
                span.X > 1e-7f ? binCount / span.X : 0,
                span.Y > 1e-7f ? binCount / span.Y : 0,
                span.Z > 1e-7f ? binCount / span.Z : 0);

            //If the span along an axis is too small, just ignore it.
            var maximumBinIndex = new Vector3(binCount - 1);

            //Initialize bin information.
            for (int i = 0; i < binCount; ++i)
            {
                resources.BinBoundingBoxesX[i] = nullBoundingBox;
                resources.BinBoundingBoxesY[i] = nullBoundingBox;
                resources.BinBoundingBoxesZ[i] = nullBoundingBox;

                resources.BinSubtreeCountsX[i] = 0;
                resources.BinSubtreeCountsY[i] = 0;
                resources.BinSubtreeCountsZ[i] = 0;

                resources.BinLeafCountsX[i] = 0;
                resources.BinLeafCountsY[i] = 0;
                resources.BinLeafCountsZ[i] = 0;
            }

            //Allocate subtrees to bins for all axes simultaneously.
            for (int i = 0; i < count; ++i)
            {
                var subtreeIndex = localIndexMap[i];
                var binIndices = Vector3.Min((resources.Centroids[subtreeIndex] - centroidBoundingBox.Min) * inverseBinSize, maximumBinIndex);
                var x = (int)binIndices.X;
                var y = (int)binIndices.Y;
                var z = (int)binIndices.Z;

                var subtreeBoundingBox = resources.BoundingBoxes + subtreeIndex;
                var leafCount = resources.LeafCounts + subtreeIndex;
                BoundingBox.Merge(ref resources.BinBoundingBoxesX[x], ref *subtreeBoundingBox, out resources.BinBoundingBoxesX[x]);
                BoundingBox.Merge(ref resources.BinBoundingBoxesY[y], ref *subtreeBoundingBox, out resources.BinBoundingBoxesY[y]);
                BoundingBox.Merge(ref resources.BinBoundingBoxesZ[z], ref *subtreeBoundingBox, out resources.BinBoundingBoxesZ[z]);
                resources.BinLeafCountsX[x] += *leafCount;
                resources.BinLeafCountsY[y] += *leafCount;
                resources.BinLeafCountsZ[z] += *leafCount;
                ++resources.BinSubtreeCountsX[x];
                ++resources.BinSubtreeCountsY[y];
                ++resources.BinSubtreeCountsZ[z];

                resources.SubtreeBinIndicesX[i] = x;
                resources.SubtreeBinIndicesY[i] = y;
                resources.SubtreeBinIndicesZ[i] = z;
            }

            //Determine split axes for all axes simultaneously.
            //Sweep from low to high.
            var lastIndex = binCount - 1;

            resources.ALeafCountsX[0] = resources.BinLeafCountsX[0];
            resources.ALeafCountsY[0] = resources.BinLeafCountsY[0];
            resources.ALeafCountsZ[0] = resources.BinLeafCountsZ[0];
            resources.AMergedX[0] = resources.BinBoundingBoxesX[0];
            resources.AMergedY[0] = resources.BinBoundingBoxesY[0];
            resources.AMergedZ[0] = resources.BinBoundingBoxesZ[0];
            for (int i = 1; i < lastIndex; ++i)
            {
                var previousIndex = i - 1;
                resources.ALeafCountsX[i] = resources.BinLeafCountsX[i] + resources.ALeafCountsX[previousIndex];
                resources.ALeafCountsY[i] = resources.BinLeafCountsY[i] + resources.ALeafCountsY[previousIndex];
                resources.ALeafCountsZ[i] = resources.BinLeafCountsZ[i] + resources.ALeafCountsZ[previousIndex];
                BoundingBox.Merge(ref resources.AMergedX[previousIndex], ref resources.BinBoundingBoxesX[i], out resources.AMergedX[i]);
                BoundingBox.Merge(ref resources.AMergedY[previousIndex], ref resources.BinBoundingBoxesY[i], out resources.AMergedY[i]);
                BoundingBox.Merge(ref resources.AMergedZ[previousIndex], ref resources.BinBoundingBoxesZ[i], out resources.AMergedZ[i]);
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


            for (int i = lastIndex; i >= 1; --i)
            {
                int aIndex = i - 1;
                BoundingBox.Merge(ref bMergedX, ref resources.BinBoundingBoxesX[i], out bMergedX);
                BoundingBox.Merge(ref bMergedY, ref resources.BinBoundingBoxesY[i], out bMergedY);
                BoundingBox.Merge(ref bMergedZ, ref resources.BinBoundingBoxesZ[i], out bMergedZ);
                bLeafCountX += resources.BinLeafCountsX[i];
                bLeafCountY += resources.BinLeafCountsY[i];
                bLeafCountZ += resources.BinLeafCountsZ[i];

                var metricAX = ComputeBoundsMetric(ref resources.AMergedX[aIndex]);
                var metricAY = ComputeBoundsMetric(ref resources.AMergedY[aIndex]);
                var metricAZ = ComputeBoundsMetric(ref resources.AMergedZ[aIndex]);
                var metricBX = ComputeBoundsMetric(ref bMergedX);
                var metricBY = ComputeBoundsMetric(ref bMergedY);
                var metricBZ = ComputeBoundsMetric(ref bMergedZ);

                //It's possible for a lot of bins in a row to be unpopulated. In that event, you'll get a metric of -infinity. Avoid letting that propagate.
                float costCandidateX, costCandidateY, costCandidateZ;
                if (metricAX > 0 && metricBX > 0)
                    costCandidateX = resources.ALeafCountsX[aIndex] * metricAX + bLeafCountX * metricBX;
                else
                    costCandidateX = float.MaxValue;
                if (metricAY > 0 && metricBY > 0)
                    costCandidateY = resources.ALeafCountsY[aIndex] * metricAY + bLeafCountY * metricBY;
                else
                    costCandidateY = float.MaxValue;
                if (metricAZ > 0 && metricBZ > 0)
                    costCandidateZ = resources.ALeafCountsZ[aIndex] * metricAZ + bLeafCountZ * metricBZ;
                else
                    costCandidateZ = float.MaxValue;
                if (costCandidateX < costCandidateY && costCandidateX < costCandidateZ)
                {
                    if (costCandidateX < cost)
                    {
                        bestAxis = 0;
                        cost = costCandidateX;
                        binSplitIndex = i;
                        a = resources.AMergedX[aIndex];
                        b = bMergedX;
                        leafCountA = resources.ALeafCountsX[aIndex];
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
                        a = resources.AMergedY[aIndex];
                        b = bMergedY;
                        leafCountA = resources.ALeafCountsY[aIndex];
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
                        a = resources.AMergedZ[aIndex];
                        b = bMergedZ;
                        leafCountA = resources.ALeafCountsZ[aIndex];
                        leafCountB = bLeafCountZ;
                    }
                }

            }


            int* bestBinSubtreeCounts;
            int* bestSubtreeBinIndices;
            switch (bestAxis)
            {
                case 0:
                    bestBinSubtreeCounts = resources.BinSubtreeCountsX;
                    bestSubtreeBinIndices = resources.SubtreeBinIndicesX;
                    break;
                case 1:
                    bestBinSubtreeCounts = resources.BinSubtreeCountsY;
                    bestSubtreeBinIndices = resources.SubtreeBinIndicesY;
                    break;
                default:
                    bestBinSubtreeCounts = resources.BinSubtreeCountsZ;
                    bestSubtreeBinIndices = resources.SubtreeBinIndicesZ;
                    break;
            }
            //Rebuild the index map.

            resources.BinStartIndices[0] = 0;
            resources.BinSubtreeCountsSecondPass[0] = 0;

            for (int i = 1; i < binCount; ++i)
            {
                resources.BinStartIndices[i] = resources.BinStartIndices[i - 1] + bestBinSubtreeCounts[i - 1];
                resources.BinSubtreeCountsSecondPass[i] = 0;
            }

            for (int i = 0; i < count; ++i)
            {
                resources.TempIndexMap[resources.BinStartIndices[bestSubtreeBinIndices[i]] + resources.BinSubtreeCountsSecondPass[bestSubtreeBinIndices[i]]++] = localIndexMap[i];
            }

            //Update the real index map.
            for (int i = 0; i < count; ++i)
            {
                localIndexMap[i] = resources.TempIndexMap[i];
            }

            //Transform the split index into object indices.
            splitIndex = resources.BinStartIndices[binSplitIndex] + start;

        }



        unsafe void SplitSubtreesIntoChildrenBinned(int depthRemaining, ref BinnedResources resources,
            int start, int count, ref BoundingBox boundingBox,
            int stagingNodeIndex, ref int stagingNodesCount, out float childrenTreeletsCost)
        {
            if (count > 1)
            {

                BoundingBox a, b;
                int leafCountA, leafCountB;
                int splitIndex;
                FindPartitionBinned(ref resources, start, count, ref boundingBox, out splitIndex, out a, out b, out leafCountA, out leafCountB);


                float costA, costB;
                if (depthRemaining > 0)
                {
                    --depthRemaining;
                    SplitSubtreesIntoChildrenBinned(depthRemaining, ref resources, start, splitIndex - start, ref a, stagingNodeIndex, ref stagingNodesCount, out costA);
                    SplitSubtreesIntoChildrenBinned(depthRemaining, ref resources, splitIndex, start + count - splitIndex, ref b, stagingNodeIndex, ref stagingNodesCount, out costB);
                }
                else
                {
                    //Recursion bottomed out. 
                    var stagingNode = resources.StagingNodes + stagingNodeIndex;
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
                        stagingChildren[childIndexA] = CreateStagingNodeBinned(stagingNodeIndex, childIndexA, ref a, ref resources, start, subtreeCountA,
                            ref stagingNodesCount, out costA);
                        costA += ComputeBoundsMetric(ref a); //An internal node was created; measure its cost.
                    }
                    else
                    {
                        Debug.Assert(subtreeCountA == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexA] = Encode(resources.IndexMap[start]);
                        costA = 0;
                    }
                    if (subtreeCountB > 1)
                    {
                        stagingChildren[childIndexB] = CreateStagingNodeBinned(stagingNodeIndex, childIndexB, ref b, ref resources, splitIndex, subtreeCountB,
                            ref stagingNodesCount, out costB);
                        costB += ComputeBoundsMetric(ref b); //An internal node was created; measure its cost.
                    }
                    else
                    {
                        Debug.Assert(subtreeCountB == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexB] = Encode(resources.IndexMap[splitIndex]);
                        costB = 0;
                    }
                }
                childrenTreeletsCost = costA + costB;
            }
            else
            {
                Debug.Assert(count == 1);
                //Only one subtree. Just stick it directly into the node.
                var childIndex = resources.StagingNodes[stagingNodeIndex].ChildCount++;
                var subtreeIndex = resources.IndexMap[start];
                Debug.Assert(resources.StagingNodes[stagingNodeIndex].ChildCount <= ChildrenCapacity);
                (&resources.StagingNodes[stagingNodeIndex].A)[childIndex] = resources.BoundingBoxes[subtreeIndex];
                (&resources.StagingNodes[stagingNodeIndex].ChildA)[childIndex] = Encode(subtreeIndex);
                (&resources.StagingNodes[stagingNodeIndex].LeafCountA)[childIndex] = resources.LeafCounts[subtreeIndex];
                //Subtrees cannot contribute to change in cost.
                childrenTreeletsCost = 0;
            }
        }

        unsafe int CreateStagingNodeBinned(int parentIndex, int indexInParent, ref BoundingBox boundingBox,
            ref BinnedResources resources, int start, int count,
            ref int stagingNodeCount, out float childTreeletsCost)
        {
            var stagingNodeIndex = stagingNodeCount++;
            var stagingNode = resources.StagingNodes + stagingNodeIndex;
            //The resource memory could contain arbitrary data.
            //ChildCount will be read, so zero it out.
            stagingNode->ChildCount = 0;

            if (count <= ChildrenCapacity)
            {
                //No need to do any sorting. This node can fit every remaining subtree.
                var localIndexMap = resources.IndexMap + start;
                stagingNode->ChildCount = count;
                var stagingNodeBounds = &stagingNode->A;
                var stagingNodeChildren = &stagingNode->ChildA;
                var leafCounts = &stagingNode->LeafCountA;
                for (int i = 0; i < count; ++i)
                {
                    var subtreeIndex = localIndexMap[i];
                    stagingNodeBounds[i] = resources.BoundingBoxes[subtreeIndex];
                    leafCounts[i] = resources.LeafCounts[subtreeIndex];
                    stagingNodeChildren[i] = Encode(subtreeIndex);
                }
                //Because subtrees do not change in size, they cannot change the cost.
                childTreeletsCost = 0;
                return stagingNodeIndex;
            }

            const int recursionDepth = ChildrenCapacity == 32 ? 4 : ChildrenCapacity == 16 ? 3 : ChildrenCapacity == 8 ? 2 : ChildrenCapacity == 4 ? 1 : 0;


            SplitSubtreesIntoChildrenBinned(recursionDepth, ref resources, start, count, ref boundingBox, stagingNodeIndex, ref stagingNodeCount, out childTreeletsCost);

            return stagingNodeIndex;

        }



        public unsafe void BinnedRefine(int nodeIndex, ref QuickList<int> spareNodes, int maximumSubtrees, ref BinnedResources resources, out bool nodesInvalidated)
        {
            var poolIndex = BufferPool<int>.GetPoolIndex(maximumSubtrees);
            var subtreeReferences = new QuickList<int>(BufferPools<int>.Thread, poolIndex);
            var treeletInternalNodes = new QuickQueue<int>(BufferPools<int>.Thread, poolIndex);
            float originalTreeletCost;
            CollectSubtrees(nodeIndex, maximumSubtrees, resources.SubtreeHeapEntries, ref subtreeReferences, ref treeletInternalNodes, out originalTreeletCost);

            //Gather necessary information from nodes.
            for (int i = 0; i < subtreeReferences.Count; ++i)
            {
                //TODO: remember to revert to pointer arith
                //var boundingBox = boundingBoxes + i;
                resources.IndexMap[i] = i;
                if (subtreeReferences.Elements[i] >= 0)
                {
                    //It's an internal node.
                    var subtreeNode = nodes + subtreeReferences.Elements[i];
                    var parentNode = nodes + subtreeNode->Parent;
                    resources.BoundingBoxes[i] = (&parentNode->A)[subtreeNode->IndexInParent];
                    resources.Centroids[i] = resources.BoundingBoxes[i].Min + resources.BoundingBoxes[i].Max;
                    resources.LeafCounts[i] = (&parentNode->LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node.
                    var leaf = leaves + Encode(subtreeReferences.Elements[i]);
                    resources.BoundingBoxes[i] = (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                    resources.Centroids[i] = resources.BoundingBoxes[i].Min + resources.BoundingBoxes[i].Max;
                    resources.LeafCounts[i] = 1;
                }
            }

            var node = nodes + nodeIndex;
            int parent = node->Parent;
            int indexInParent = node->IndexInParent;

            //Now perform a top-down sweep build.
            //TODO: this staging creation section is really the only part that is sweep-specific. The rest is common to any other kind of subtree-collection based refinement. 
            //If you end up making others, keep this in mind.
            int stagingNodeCount = 0;

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
            CreateStagingNodeBinned(parent, indexInParent, ref treeletBoundingBox, ref resources, 0, subtreeReferences.Count, ref stagingNodeCount, out newTreeletCost);


            //ValidateStaging(stagingNodes, sweepSubtrees, ref subtreeReferences, parent, indexInParent);

            if (true)//newTreeletCost < originalTreeletCost)
            {
                //The refinement is an actual improvement.
                //Apply the staged nodes to real nodes!
                var reifiedIndex = ReifyStagingNode(parent, indexInParent, resources.StagingNodes, 0, ref subtreeReferences, ref treeletInternalNodes, ref spareNodes, out nodesInvalidated);
                Debug.Assert(parent != -1 ? (&nodes[parent].ChildA)[indexInParent] == reifiedIndex : true, "The parent should agree with the child about the relationship.");
                //If any nodes are left over, put them into the spares list for later reuse.
                int spareNode;
                while (treeletInternalNodes.TryDequeue(out spareNode))
                {
                    spareNodes.Add(spareNode);
                }
            }
            else
            {
                nodesInvalidated = false;
            }

            subtreeReferences.Dispose();
            treeletInternalNodes.Dispose();

        }





        private unsafe void TopDownBinnedRefine(int nodeIndex, int maximumSubtrees, ref QuickList<int> spareNodes, ref BinnedResources resources)
        {
            bool nodesInvalidated;
            //Validate();
            BinnedRefine(nodeIndex, ref spareNodes, maximumSubtrees, ref resources, out nodesInvalidated);
            //Validate();
            //The root of the tree is guaranteed to stay in position, so nodeIndex is still valid.

            //Node pointers can be invalidated, so don't hold a reference between executions.
            for (int i = 0; i < nodes[nodeIndex].ChildCount; ++i)
            {
                var child = (&nodes[nodeIndex].ChildA)[i];
                if (child >= 0)
                {
                    TopDownBinnedRefine(child, maximumSubtrees, ref spareNodes, ref resources);
                }
            }
        }
        public unsafe void TopDownBinnedRefine(int maximumSubtrees)
        {
            var pool = BufferPools<int>.Thread;

            var spareNodes = new QuickList<int>(pool, 8);
            MemoryRegion region;
            BinnedResources resources;
            CreateBinnedResources(pool, maximumSubtrees, out region, out resources);
            TopDownBinnedRefine(0, maximumSubtrees, ref spareNodes, ref resources);
            RemoveUnusedInternalNodes(ref spareNodes);
            region.Dispose();
            spareNodes.Dispose();
        }


        unsafe void TryToBottomUpBinnedRefine(int[] refinementFlags, int nodeIndex, int maximumSubtrees, ref BinnedResources resources, ref QuickList<int> spareInternalNodes)
        {
            if (++refinementFlags[nodeIndex] == nodes[nodeIndex].ChildCount)
            {
                bool nodesInvalidated;
                BinnedRefine(nodeIndex, ref spareInternalNodes, maximumSubtrees, ref resources, out nodesInvalidated);

                var parent = nodes[nodeIndex].Parent;
                if (parent != -1)
                {
                    TryToBottomUpBinnedRefine(refinementFlags, parent, maximumSubtrees, ref resources, ref spareInternalNodes);
                }
            }
        }


        /// <summary>
        /// Executes one pass of bottom-up refinement.
        /// </summary>
        public unsafe void BottomUpBinnedRefine(int maximumSubtrees)
        {
            //If this works out, should probably choose a more efficient flagging approach.
            //Note the size: it needs to contain all possible internal nodes.
            //TODO: This is actually bugged, because the refinement flags do not update if the nodes move.
            //And the nodes CAN move.
            var pool = BufferPools<int>.Thread;

            var spareNodes = new QuickList<int>(pool, 8);
            MemoryRegion region;
            BinnedResources resources;
            CreateBinnedResources(pool, maximumSubtrees, out region, out resources);
            var refinementFlags = new int[leafCount * 2 - 1];
            for (int i = 0; i < nodeCount; ++i)
            {
                refinementFlags[i] = 0;
            }
            for (int i = 0; i < leafCount; ++i)
            {
                TryToBottomUpBinnedRefine(refinementFlags, leaves[i].NodeIndex, maximumSubtrees, ref resources, ref spareNodes);
                //Validate();
            }
            //Console.WriteLine($"root children: {nodes->ChildCount}");
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
            region.Dispose();
        }
    }
}
