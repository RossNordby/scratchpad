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
                16 * (3 + 3 + 1) + sizeof(BoundingBox) * (maximumSubtreeCount + 3 * nodeCount + 3 * maximumBinCount) +
                16 * (6 + 3 + 8) + sizeof(int) * (maximumSubtreeCount * 6 + nodeCount * 3 + maximumBinCount * 8) +
                16 * (1) + sizeof(Vector3) * maximumSubtreeCount +
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


        unsafe void FindPartitionBinned(ref BinnedResources subtrees, int start, int count, ref BoundingBox boundingBox,
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
                subtrees.BinBoundingBoxesX[i] = nullBoundingBox;
                subtrees.BinBoundingBoxesY[i] = nullBoundingBox;
                subtrees.BinBoundingBoxesZ[i] = nullBoundingBox;

                subtrees.BinSubtreeCountsX[i] = 0;
                subtrees.BinSubtreeCountsY[i] = 0;
                subtrees.BinSubtreeCountsZ[i] = 0;

                subtrees.BinLeafCountsX[i] = 0;
                subtrees.BinLeafCountsY[i] = 0;
                subtrees.BinLeafCountsZ[i] = 0;
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
                BoundingBox.Merge(ref subtrees.BinBoundingBoxesX[x], ref *subtreeBoundingBox, out subtrees.BinBoundingBoxesX[x]);
                BoundingBox.Merge(ref subtrees.BinBoundingBoxesY[y], ref *subtreeBoundingBox, out subtrees.BinBoundingBoxesY[y]);
                BoundingBox.Merge(ref subtrees.BinBoundingBoxesZ[z], ref *subtreeBoundingBox, out subtrees.BinBoundingBoxesZ[z]);
                subtrees.BinLeafCountsX[x] += *leafCount;
                subtrees.BinLeafCountsY[y] += *leafCount;
                subtrees.BinLeafCountsZ[z] += *leafCount;
                ++subtrees.BinSubtreeCountsX[x];
                ++subtrees.BinSubtreeCountsY[y];
                ++subtrees.BinSubtreeCountsZ[z];

                subtrees.SubtreeBinIndicesX[i] = x;
                subtrees.SubtreeBinIndicesY[i] = y;
                subtrees.SubtreeBinIndicesZ[i] = z;
            }

            //Determine split axes for all axes simultaneously.
            //Sweep from low to high.
            var lastIndex = binCount - 1;

            subtrees.ALeafCountsX[0] = subtrees.BinLeafCountsX[0];
            subtrees.ALeafCountsY[0] = subtrees.BinLeafCountsY[0];
            subtrees.ALeafCountsZ[0] = subtrees.BinLeafCountsZ[0];
            subtrees.AMergedX[0] = subtrees.BinBoundingBoxesX[0];
            subtrees.AMergedY[0] = subtrees.BinBoundingBoxesY[0];
            subtrees.AMergedZ[0] = subtrees.BinBoundingBoxesZ[0];
            for (int i = 1; i < lastIndex; ++i)
            {
                var previousIndex = i - 1;
                subtrees.ALeafCountsX[i] = subtrees.BinLeafCountsX[i] + subtrees.ALeafCountsX[previousIndex];
                subtrees.ALeafCountsY[i] = subtrees.BinLeafCountsY[i] + subtrees.ALeafCountsY[previousIndex];
                subtrees.ALeafCountsZ[i] = subtrees.BinLeafCountsZ[i] + subtrees.ALeafCountsZ[previousIndex];
                BoundingBox.Merge(ref subtrees.AMergedX[previousIndex], ref subtrees.BinBoundingBoxesX[i], out subtrees.AMergedX[i]);
                BoundingBox.Merge(ref subtrees.AMergedY[previousIndex], ref subtrees.BinBoundingBoxesY[i], out subtrees.AMergedY[i]);
                BoundingBox.Merge(ref subtrees.AMergedZ[previousIndex], ref subtrees.BinBoundingBoxesZ[i], out subtrees.AMergedZ[i]);
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
                BoundingBox.Merge(ref bMergedX, ref subtrees.BinBoundingBoxesX[i], out bMergedX);
                BoundingBox.Merge(ref bMergedY, ref subtrees.BinBoundingBoxesY[i], out bMergedY);
                BoundingBox.Merge(ref bMergedZ, ref subtrees.BinBoundingBoxesZ[i], out bMergedZ);
                bLeafCountX += subtrees.BinLeafCountsX[i];
                bLeafCountY += subtrees.BinLeafCountsY[i];
                bLeafCountZ += subtrees.BinLeafCountsZ[i];

                var metricAX = ComputeBoundsMetric(ref subtrees.AMergedX[aIndex]);
                var metricAY = ComputeBoundsMetric(ref subtrees.AMergedY[aIndex]);
                var metricAZ = ComputeBoundsMetric(ref subtrees.AMergedZ[aIndex]);
                var metricBX = ComputeBoundsMetric(ref bMergedX);
                var metricBY = ComputeBoundsMetric(ref bMergedY);
                var metricBZ = ComputeBoundsMetric(ref bMergedZ);

                //It's possible for a lot of bins in a row to be unpopulated. In that event, you'll get a metric of -infinity. Avoid letting that propagate.
                float costCandidateX, costCandidateY, costCandidateZ;
                if (metricAX > 0 && metricBX > 0)
                    costCandidateX = subtrees.ALeafCountsX[aIndex] * metricAX + bLeafCountX * metricBX;
                else
                    costCandidateX = float.MaxValue;
                if (metricAY > 0 && metricBY > 0)
                    costCandidateY = subtrees.ALeafCountsY[aIndex] * metricAY + bLeafCountY * metricBY;
                else
                    costCandidateY = float.MaxValue;
                if (metricAZ > 0 && metricBZ > 0)
                    costCandidateZ = subtrees.ALeafCountsZ[aIndex] * metricAZ + bLeafCountZ * metricBZ;
                else
                    costCandidateZ = float.MaxValue;
                if (costCandidateX < costCandidateY && costCandidateX < costCandidateZ)
                {
                    if (costCandidateX < cost)
                    {
                        bestAxis = 0;
                        cost = costCandidateX;
                        binSplitIndex = i;
                        a = subtrees.AMergedX[aIndex];
                        b = bMergedX;
                        leafCountA = subtrees.ALeafCountsX[aIndex];
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
                        a = subtrees.AMergedY[aIndex];
                        b = bMergedY;
                        leafCountA = subtrees.ALeafCountsY[aIndex];
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
                        a = subtrees.AMergedZ[aIndex];
                        b = bMergedZ;
                        leafCountA = subtrees.ALeafCountsZ[aIndex];
                        leafCountB = bLeafCountZ;
                    }
                }

            }


            int* bestBinSubtreeCounts;
            int* bestSubtreeBinIndices;
            switch (bestAxis)
            {
                case 0:
                    bestBinSubtreeCounts = subtrees.BinSubtreeCountsX;
                    bestSubtreeBinIndices = subtrees.SubtreeBinIndicesX;
                    break;
                case 1:
                    bestBinSubtreeCounts = subtrees.BinSubtreeCountsY;
                    bestSubtreeBinIndices = subtrees.SubtreeBinIndicesY;
                    break;
                default:
                    bestBinSubtreeCounts = subtrees.BinSubtreeCountsZ;
                    bestSubtreeBinIndices = subtrees.SubtreeBinIndicesZ;
                    break;
            }
            //Rebuild the index map.

            subtrees.BinStartIndices[0] = 0;
            subtrees.BinSubtreeCountsSecondPass[0] = 0;

            for (int i = 1; i < binCount; ++i)
            {
                subtrees.BinStartIndices[i] = subtrees.BinStartIndices[i - 1] + bestBinSubtreeCounts[i - 1];
                subtrees.BinSubtreeCountsSecondPass[i] = 0;
            }

            for (int i = 0; i < count; ++i)
            {
                subtrees.TempIndexMap[subtrees.BinStartIndices[bestSubtreeBinIndices[i]] + subtrees.BinSubtreeCountsSecondPass[bestSubtreeBinIndices[i]]++] = localIndexMap[i];
            }

            //Update the real index map.
            for (int i = 0; i < count; ++i)
            {
                localIndexMap[i] = subtrees.TempIndexMap[i];
            }

            //Transform the split index into object indices.
            splitIndex = subtrees.BinStartIndices[binSplitIndex] + start;

        }



        unsafe void SplitSubtreesIntoChildrenBinned(int depthRemaining, ref BinnedResources subtrees,
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
            ref BinnedResources subtrees, int start, int count,
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


            SplitSubtreesIntoChildrenBinned(recursionDepth, ref subtrees, start, count, ref boundingBox, stagingNodes, stagingNodeIndex, ref stagingNodeCount, out childTreeletsCost);

            return stagingNodeIndex;

        }



        public unsafe void BinnedRefine(int nodeIndex, ref QuickList<int> spareNodes, int maximumSubtrees, ref BinnedResources subtrees, out bool nodesInvalidated)
        {
            var poolIndex = BufferPool<int>.GetPoolIndex(maximumSubtrees);
            var subtreeReferences = new QuickList<int>(BufferPools<int>.Thread, poolIndex);
            var treeletInternalNodes = new QuickQueue<int>(BufferPools<int>.Thread, poolIndex);
            float originalTreeletCost;
            CollectSubtrees(nodeIndex, maximumSubtrees, ref subtreeReferences, ref treeletInternalNodes, out originalTreeletCost);


            //Create the resources that will be shared during the refinement process.
            //Some awkwardness: can't stackalloc into anything but a local.
            //If this seems like a lot, ... well it is. It puts a pretty hard cap
            //on the size of refinement operations unless you fiddle with the stack size.
            //Doing all this upfront saves zeroing time (at least until stackalloc is improved).
            var boundingBoxes = stackalloc BoundingBox[subtreeReferences.Count];
            var leafCounts = stackalloc int[subtreeReferences.Count];
            var indexMap = stackalloc int[subtreeReferences.Count];
            var centroids = stackalloc Vector3[subtreeReferences.Count];

            var subtreeBinIndicesX = stackalloc int[subtreeReferences.Count];
            var subtreeBinIndicesY = stackalloc int[subtreeReferences.Count];
            var subtreeBinIndicesZ = stackalloc int[subtreeReferences.Count];
            var tempIndexMap = stackalloc int[subtreeReferences.Count];

            var aLeafCountsX = stackalloc int[subtreeReferences.Count - 1];
            var aLeafCountsY = stackalloc int[subtreeReferences.Count - 1];
            var aLeafCountsZ = stackalloc int[subtreeReferences.Count - 1];
            var aMergedX = stackalloc BoundingBox[subtreeReferences.Count - 1];
            var aMergedY = stackalloc BoundingBox[subtreeReferences.Count - 1];
            var aMergedZ = stackalloc BoundingBox[subtreeReferences.Count - 1];

            var binBoundingBoxesX = stackalloc BoundingBox[MaximumBinCount];
            var binBoundingBoxesY = stackalloc BoundingBox[MaximumBinCount];
            var binBoundingBoxesZ = stackalloc BoundingBox[MaximumBinCount];
            var binLeafCountsX = stackalloc int[MaximumBinCount];
            var binLeafCountsY = stackalloc int[MaximumBinCount];
            var binLeafCountsZ = stackalloc int[MaximumBinCount];
            var binSubtreeCountsX = stackalloc int[MaximumBinCount];
            var binSubtreeCountsY = stackalloc int[MaximumBinCount];
            var binSubtreeCountsZ = stackalloc int[MaximumBinCount];
            var binStartIndices = stackalloc int[MaximumBinCount];
            var binSubtreeCountsSecondPass = stackalloc int[MaximumBinCount];



            BinnedResources subtrees;
            subtrees.BoundingBoxes = boundingBoxes;
            subtrees.LeafCounts = leafCounts;
            subtrees.IndexMap = indexMap;
            subtrees.Centroids = centroids;

            subtrees.SubtreeBinIndicesX = subtreeBinIndicesX;
            subtrees.SubtreeBinIndicesY = subtreeBinIndicesY;
            subtrees.SubtreeBinIndicesZ = subtreeBinIndicesZ;
            subtrees.TempIndexMap = tempIndexMap;

            subtrees.ALeafCountsX = aLeafCountsX;
            subtrees.ALeafCountsY = aLeafCountsY;
            subtrees.ALeafCountsZ = aLeafCountsZ;
            subtrees.AMergedX = aMergedX;
            subtrees.AMergedY = aMergedY;
            subtrees.AMergedZ = aMergedZ;

            subtrees.BinBoundingBoxesX = binBoundingBoxesX;
            subtrees.BinBoundingBoxesY = binBoundingBoxesY;
            subtrees.BinBoundingBoxesZ = binBoundingBoxesZ;
            subtrees.BinLeafCountsX = binLeafCountsX;
            subtrees.BinLeafCountsY = binLeafCountsY;
            subtrees.BinLeafCountsZ = binLeafCountsZ;
            subtrees.BinSubtreeCountsX = binSubtreeCountsX;
            subtrees.BinSubtreeCountsY = binSubtreeCountsY;
            subtrees.BinSubtreeCountsZ = binSubtreeCountsZ;
            subtrees.BinStartIndices = binStartIndices;
            subtrees.BinSubtreeCountsSecondPass = binSubtreeCountsSecondPass;
            //Gather necessary information from nodes.
            for (int i = 0; i < subtreeReferences.Count; ++i)
            {
                //TODO: remember to revert to pointer arith
                //var boundingBox = boundingBoxes + i;
                indexMap[i] = i;
                if (subtreeReferences.Elements[i] >= 0)
                {
                    //It's an internal node.
                    var subtreeNode = nodes + subtreeReferences.Elements[i];
                    var parentNode = nodes + subtreeNode->Parent;
                    boundingBoxes[i] = (&parentNode->A)[subtreeNode->IndexInParent];
                    centroids[i] = boundingBoxes[i].Min + boundingBoxes[i].Max;
                    leafCounts[i] = (&parentNode->LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node.
                    var leaf = leaves + Encode(subtreeReferences.Elements[i]);
                    boundingBoxes[i] = (&nodes[leaf->NodeIndex].A)[leaf->ChildIndex];
                    centroids[i] = boundingBoxes[i].Min + boundingBoxes[i].Max;
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
            subtrees.StagingNodes = stagingNodes;

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
            CreateStagingNodeBinned(parent, indexInParent, ref treeletBoundingBox, ref subtrees, 0, subtreeReferences.Count, subtrees.StagingNodes, ref stagingNodeCount, out newTreeletCost);


            //ValidateStaging(stagingNodes, sweepSubtrees, ref subtreeReferences, parent, indexInParent);

            if (newTreeletCost < originalTreeletCost)
            {
                //The refinement is an actual improvement.
                //Apply the staged nodes to real nodes!
                var reifiedIndex = ReifyStagingNode(parent, indexInParent, stagingNodes, 0, ref subtreeReferences, ref treeletInternalNodes, ref spareNodes, out nodesInvalidated);
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
            adsf
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
            TopDownBinnedRefine(0, maximumSubtrees, ref spareNodes, ref resources);
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
