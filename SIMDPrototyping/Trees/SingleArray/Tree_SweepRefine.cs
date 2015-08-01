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
            public int LeafCount;
        }

      

        unsafe void FindPartitionForAxis(SweepSubtree* subtrees, int* indexMap, Vector3* centroids, int subtreeCount, int axisIndex,
            out int splitIndex, out float cost, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            Debug.Assert(subtreeCount > 1);
            //TODO: Test performance of accessing centroid components by reinterpreting the centroid pointer as a float and using a stride with offset.
            //Another option: store as SOA to begin with. Frontloads the cost of transposition and avoids all access difficulty. My guess is this will be a win.

            //Sort the index map according to the centroids.
            for (int i = 1; i < subtreeCount; ++i)
            {
                var index = i;
                var previousIndex = index - 1;
                while ((&centroids[indexMap[index]].X)[axisIndex] < (&centroids[indexMap[previousIndex]].X)[axisIndex])
                {

                    var tempPointer = indexMap[index];
                    indexMap[index] = indexMap[previousIndex];
                    indexMap[previousIndex] = tempPointer;


                    if (previousIndex == 0)
                        break;
                    index = previousIndex;
                    --previousIndex;
                }
            }

            //Search for the best split.
            //Sweep across from low to high, caching the merged size and leaf count at each point.
            //Index N includes every subtree from 0 to N, inclusive. So index 0 contains subtree 0's information.
            var lastIndex = subtreeCount - 1;
            var aLeafCounts = stackalloc int[lastIndex];
            var aMerged = stackalloc BoundingBox[lastIndex];
            aLeafCounts[0] = subtrees[0].LeafCount;
            aMerged[0] = subtrees[0].BoundingBox;
            for (int i = 1; i < subtreeCount; ++i)
            {
                aLeafCounts[i] = subtrees[i].LeafCount + aLeafCounts[i - 1];
                BoundingBox.Merge(ref aMerged[i - 1], ref subtrees[i].BoundingBox, out aMerged[i]);
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
            for (int i = lastIndex - 1; i >= 1; --i)
            {
                int aIndex = i - 1;
                BoundingBox.Merge(ref bMerged, ref subtrees[lastIndex].BoundingBox, out bMerged);
                bLeafCount += subtrees[lastIndex].LeafCount;

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
        unsafe void FindPartition(SweepSubtree* subtrees, int* indexMap, int subtreeStart, int subtreeCount, ref BoundingBox boundingBox,
               out int splitIndex, out float cost, out BoundingBox a, out BoundingBox b, out int leafCountA, out int leafCountB)
        {
            //Compute dominant axis based on the bounding box.
            //Sort along that axis. This uses an in-place insertion sort for simplicity and to take advantage of the partially sorted data.
            var offset = boundingBox.Max - boundingBox.Min;

            //A variety of potential microoptimizations exist here.
            //Don't reallocate centroids (because JIT is forced to zero by roslyn), do swaps better, etc.
            var centroids = stackalloc Vector3[subtreeCount];
            var indexMapX = stackalloc int[subtreeCount];
            var indexMapY = stackalloc int[subtreeCount];
            var indexMapZ = stackalloc int[subtreeCount];

            //Initialize the per-axis candidate maps.
            subtrees += subtreeStart;
            indexMap += subtreeStart;
            for (int i = 0; i < subtreeCount; ++i)
            {
                var originalValue = indexMap[i];
                indexMapX[i] = originalValue;
                indexMapY[i] = originalValue;
                indexMapZ[i] = originalValue;
                centroids[i] = subtrees[i].BoundingBox.Min + subtrees[i].BoundingBox.Max;
            }

            int xSplitIndex, xLeafCountA, xLeafCountB, ySplitIndex, yLeafCountA, yLeafCountB, zSplitIndex, zLeafCountA, zLeafCountB;
            BoundingBox xA, xB, yA, yB, zA, zB;
            float xCost, yCost, zCost;
            FindPartitionForAxis(subtrees, indexMapX, centroids, subtreeCount, 0, out xSplitIndex, out xCost, out xA, out xB, out xLeafCountA, out xLeafCountB);
            FindPartitionForAxis(subtrees, indexMapY, centroids, subtreeCount, 0, out ySplitIndex, out yCost, out yA, out yB, out yLeafCountA, out yLeafCountB);
            FindPartitionForAxis(subtrees, indexMapZ, centroids, subtreeCount, 0, out zSplitIndex, out zCost, out zA, out zB, out zLeafCountA, out zLeafCountB);

            if (xCost <= yCost && xCost <= zCost)
            {
                splitIndex = xSplitIndex;
                cost = xCost;
                a = xA;
                b = xB;
                leafCountA = xLeafCountA;
                leafCountB = xLeafCountB;
            }
            else if (yCost <= zCost)
            {
                splitIndex = ySplitIndex;
                cost = yCost;
                a = yA;
                b = yB;
                leafCountA = yLeafCountA;
                leafCountB = yLeafCountB;
            }
            else
            {
                splitIndex = zSplitIndex;
                cost = zCost;
                a = zA;
                b = zB;
                leafCountA = zLeafCountA;
                leafCountB = zLeafCountB;
            }
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
                    var stagingLeafCounts = &stagingNode->LeafCountA;

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
                        costA += ComputeBoundsMetric(ref a); //An internal node was created; measure its cost.
                    }
                    else
                    {
                        Debug.Assert(subtreeCountA == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexA] = Encode(subtrees[subtreeStart].Index);
                        costA = 0;
                    }
                    if (subtreeCountB > 1)
                    {
                        stagingChildren[childIndexB] = CreateStagingNode(stagingNodeIndex, childIndexB, ref b, subtrees, splitIndex, subtreeCountB,
                            stagingNodes, ref stagingNodesCount, out costB);
                        costB += ComputeBoundsMetric(ref b); //An internal node was created; measure its cost.
                    }
                    else
                    {
                        Debug.Assert(subtreeCountB == 1);
                        //Only one subtree. Don't create another node.
                        stagingChildren[childIndexB] = Encode(subtrees[splitIndex].Index);
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
                (&stagingNodes[stagingNodeIndex].ChildA)[childIndex] = Encode(subtrees[subtreeStart].Index);
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
                    stagingNodeChildren[i] = Encode(subtrees[i].Index);
                }
                //Because subtrees do not change in size, they cannot change the cost.
                childTreeletsCost = 0;
                return stagingNodeIndex;
            }

            const int recursionDepth = ChildrenCapacity == 32 ? 4 : ChildrenCapacity == 16 ? 3 : ChildrenCapacity == 8 ? 2 : ChildrenCapacity == 4 ? 1 : 0;


            SplitSubtreesIntoChildren(recursionDepth, subtrees, subtreeStart, subtreeCount, ref boundingBox, stagingNodes, stagingNodeIndex, ref stagingNodeCount, out childTreeletsCost);


            return stagingNodeIndex;

        }



        public unsafe void SweepRefine(int nodeIndex, ref QuickList<int> internalNodes, out bool nodesInvalidated)
        {
            const int maximumSubtrees = 256;
            var subtrees = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            int internalNodeStartIndex = internalNodes.Count;
            float originalTreeletCost;

            CollectSubtrees(nodeIndex, maximumSubtrees, ref subtrees, ref internalNodes, out originalTreeletCost);


            //Gather necessary information from nodes. (TODO: This could be more efficiently gathered up front... collectsubtrees already touched most of this data!)
            var sweepSubtrees = stackalloc SweepSubtree[subtrees.Count];
            for (int i = 0; i < subtrees.Count; ++i)
            {
                var subtree = sweepSubtrees + i;
                subtree->Index = i;
                if (subtrees.Elements[i] >= 0)
                {
                    //It's an internal node.
                    var subtreeNode = nodes + subtrees.Elements[i];
                    var parentNode = nodes + subtreeNode->Parent;
                    subtree->BoundingBox = (&parentNode->A)[subtreeNode->IndexInParent];
                    subtree->LeafCount = (&parentNode->LeafCountA)[subtreeNode->IndexInParent];
                }
                else
                {
                    //It's a leaf node.
                    subtree->LeafCount = 1;
                    var leaf = leaves + Encode(subtrees.Elements[i]);
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

            //ValidateStaging(stagingNodes, sweepSubtrees, ref subtrees, parent, indexInParent);

            if (newTreeletCost < originalTreeletCost)
            {
                //Reify the nodes.
                //ValidateLeaves();
                ReifyStagingNode(parent, indexInParent, stagingNodes, 0, stagingNodeCapacity, ref subtrees, ref internalNodes, out nodesInvalidated);
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

            subtrees.Dispose();

        }



        unsafe void ValidateStaging(Node* stagingNodes, SweepSubtree* subtrees, ref QuickList<int> subtreeNodePointers, int treeletParent, int treeletIndexInParent)
        {
            int foundSubtrees, foundLeafCount;
            QuickList<int> collectedSubtreeReferences = new QuickList<int>(BufferPools<int>.Thread);
            ValidateStaging(stagingNodes, 0, subtrees, ref subtreeNodePointers, ref collectedSubtreeReferences, out foundSubtrees, out foundLeafCount);
            if (treeletParent < -1 || treeletParent >= nodeCount)
                throw new Exception("Bad treelet parent.");
            if (treeletIndexInParent < -1 || (treeletParent >= 0 && treeletIndexInParent >= nodes[treeletParent].ChildCount))
                throw new Exception("Bad treelet index in parent.");
            if (treeletParent >= 0 && (&nodes[treeletParent].LeafCountA)[treeletIndexInParent] != foundLeafCount)
            {
                throw new Exception("Bad leaf count.");
            }
            if (subtreeNodePointers.Count != foundSubtrees)
            {
                throw new Exception("Bad subtree found count.");
            }
            for (int i = 0; i < collectedSubtreeReferences.Count; ++i)
            {
                if (!subtreeNodePointers.Contains(collectedSubtreeReferences[i]) || !collectedSubtreeReferences.Contains(subtreeNodePointers[i]))
                    throw new Exception("Bad subtree reference.");
            }
            collectedSubtreeReferences.Dispose();
        }
        unsafe void ValidateStaging(Node* stagingNodes, int stagingNodeIndex, SweepSubtree* subtrees, ref QuickList<int> subtreeNodePointers, ref QuickList<int> collectedSubtreeReferences, out int foundSubtrees, out int foundLeafCount)
        {
            var stagingNode = stagingNodes + stagingNodeIndex;
            var children = &stagingNode->ChildA;
            var leafCounts = &stagingNode->LeafCountA;
            foundSubtrees = foundLeafCount = 0;
            for (int i = 0; i < stagingNode->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    int childFoundSubtrees, childFoundLeafCount;
                    ValidateStaging(stagingNodes, children[i], subtrees, ref subtreeNodePointers, ref collectedSubtreeReferences, out childFoundSubtrees, out childFoundLeafCount);
                    if (childFoundLeafCount != leafCounts[i])
                        throw new Exception("Bad leaf count.");
                    foundSubtrees += childFoundSubtrees;
                    foundLeafCount += childFoundLeafCount;
                }
                else
                {
                    var subtreeNodePointerIndex = Encode(children[i]);
                    var subtreeNodePointer = subtreeNodePointers.Elements[subtreeNodePointerIndex];
                    //Rather than looking up the shuffled SweepSubtree for information, just go back to the source.
                    if (subtreeNodePointer >= 0)
                    {
                        var node = nodes + subtreeNodePointer;
                        var totalLeafCount = 0;
                        for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
                        {
                            totalLeafCount += (&node->LeafCountA)[childIndex];
                        }

                        if (leafCounts[i] != totalLeafCount)
                            throw new Exception("bad leaf count.");
                        foundLeafCount += totalLeafCount;
                    }
                    else
                    {
                        var leafIndex = Encode(subtreeNodePointer);
                        if (leafCounts[i] != 1)
                            throw new Exception("bad leaf count.");
                        foundLeafCount += 1;
                    }
                    ++foundSubtrees;
                    collectedSubtreeReferences.Add(subtreeNodePointer);
                }
            }

        }


        private unsafe void TopDownSweepRefine(int nodeIndex, ref QuickList<int> spareNodes)
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
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
            TopDownSweepRefine(0, ref spareNodes);
            RemoveUnusedInternalNodes(ref spareNodes);
            spareNodes.Dispose();
        }


        unsafe void TryToBottomUpSweepRefine(int[] refinementFlags, int nodeIndex, ref QuickList<int> spareInternalNodes)
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
            var spareNodes = new QuickList<int>(BufferPools<int>.Thread, 8);
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
