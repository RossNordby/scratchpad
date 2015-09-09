using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

using SpinLock = BEPUutilities.SpinLock;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {



        unsafe void CollectNodesForMultithreadedRefit(int nodeIndex, int leafCountThreshold, ref QuickList<int> targets)
        {
            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            Debug.Assert(node->RefineFlag == 0);
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    //Each node stores how many children are involved in the multithreaded refit.
                    //This allows the postphase to climb the tree in a thread safe way.
                    ++node->RefineFlag;
                    if (leafCounts[i] <= leafCountThreshold)
                    {
                        targets.Add(children[i]);
                    }
                    else
                    {
                        CollectNodesForMultithreadedRefit(children[i], leafCountThreshold, ref targets);
                    }
                }
            }
        }


        void CollectNodesForMultithreadedRefit(int threadCount, ref QuickList<int> targets)
        {
            //No point in using this if there aren't enough leaves.
            Debug.Assert(leafCount > 2);
            int leafCountThreshold = leafCount / (threadCount * 2);
            CollectNodesForMultithreadedRefit(0, leafCountThreshold, ref targets);
        }

        /// <summary>
        /// Caches input and output for the multithreaded execution of a tree's refit and refinement operations.
        /// </summary>
        public class RefitAndRefineMultithreadedContext
        {
            public Tree Tree;

            public int RefitNodeIndex;
            public QuickList<int> RefitNodes;
            public float RefitVolumeChange;
            public int LeafCountThreshold;
            public RawList<QuickList<int>> RefinementCandidates;
            public Action<int> RefitAndMarkAction;

            public QuickList<int> RefinementTargets;
            public BufferPool<int> Pool;
            public int MaximumSubtrees;
            public Action<int> RefineAction;

            public QuickList<int> CacheOptimizeStarts;
            public int CacheOptimizeCount;
            public Action<int> CacheOptimizeAction;

            public RefitAndRefineMultithreadedContext(Tree tree)
            {
                Tree = tree;
                RefitAndMarkAction = RefitAndMark;
                RefineAction = Refine;
                CacheOptimizeAction = CacheOptimize;
            }

            public void Initialize(int workerCount, BufferPool<int> pool)
            {
                RefitNodeIndex = -1;
                if (RefinementCandidates.Capacity < workerCount)
                    RefinementCandidates.Capacity = workerCount;
                Pool = pool;
            }

            public void CleanUp()
            {
                RefitNodes.Dispose();
                for (int i = 0; i < RefinementCandidates.Count; ++i)
                {
                    RefinementCandidates.Elements[i].Count = 0;
                    RefinementCandidates.Elements[i].Dispose();
                }
                RefinementCandidates.Clear();

                CacheOptimizeStarts.Count = 0;
                CacheOptimizeStarts.Dispose();

            }

            unsafe void RefitAndMark(int workerIndex)
            {
                int refitNodeIndex;
                while ((refitNodeIndex = Interlocked.Increment(ref RefitNodeIndex)) < RefitNodes.Count)
                {

                    var node = Tree.nodes + RefitNodes.Elements[refitNodeIndex];
                    Debug.Assert(node->Parent >= 0, "The root should not be marked for refit.");
                    var parent = Tree.nodes + node->Parent;
                    var boundingBoxInParent = &parent->A + node->IndexInParent;
                    node->LocalCostChange = Tree.RefitAndMark(RefitNodes.Elements[refitNodeIndex], LeafCountThreshold, ref RefinementCandidates.Elements[workerIndex], ref *boundingBoxInParent);

                    //Walk up the tree.
                    node = parent;
                    while (true)
                    {

                        if (Interlocked.Decrement(ref node->RefineFlag) == 0)
                        {
                            //Compute the child contributions to this node's volume change.
                            var children = &node->ChildA;
                            node->LocalCostChange = 0;
                            for (int i = 0; i < node->ChildCount; ++i)
                            {
                                if (children[i] >= 0)
                                {
                                    var child = Tree.nodes + children[i];
                                    node->LocalCostChange += child->LocalCostChange;
                                    //Clear the refine flag (unioned).
                                    child->RefineFlag = 0;
                                }
                            }

                            //This thread is the last thread to visit this node, so it must handle this node.
                            //Merge all the child bounding boxes into one. 
                            if (node->Parent < 0)
                            {
                                //Root node.
                                //Don't bother including the root's change in volume.
                                //Refinement can't change the root's bounds, so the fact that the world got bigger or smaller
                                //doesn't really have any bearing on how much refinement should be done.
                                RefitVolumeChange = node->LocalCostChange;
                                //Clear the root's refine flag (unioned).
                                node->RefineFlag = 0;
                                return;
                            }
                            else
                            {
                                parent = Tree.nodes + node->Parent;
                                boundingBoxInParent = &parent->A + node->IndexInParent;
                                var premetric = ComputeBoundsMetric(ref *boundingBoxInParent);
                                *boundingBoxInParent = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
                                var bounds = &node->A;
                                for (int i = 0; i < node->ChildCount; ++i)
                                {
                                    BoundingBox.Merge(ref bounds[i], ref *boundingBoxInParent, out *boundingBoxInParent);
                                }
                                var postmetric = ComputeBoundsMetric(ref *boundingBoxInParent);
                                node->LocalCostChange += postmetric - premetric;
                                node = parent;
                            }
                        }
                        else
                        {
                            //This thread wasn't the last to visit this node, so it should die. Some other thread will handle it later.
                            return;
                        }
                    }
                }


            }

            unsafe void Refine(int workerIndex)
            {
                var start = RefinementStarts.Elements[workerIndex];
                var end = workerIndex + 1 < RefinementStarts.Count ? RefinementStarts.Elements[workerIndex + 1] : RefinementTargets.Count;

                var spareNodes = new QuickList<int>(Pool, 8);
                var subtreeReferences = new QuickList<int>(Pool, BufferPool<int>.GetPoolIndex(MaximumSubtrees));
                var treeletInternalNodes = new QuickList<int>(Pool, BufferPool<int>.GetPoolIndex(MaximumSubtrees));
                int[] buffer;
                MemoryRegion region;
                BinnedResources resources;
                CreateBinnedResources(Pool, MaximumSubtrees, out buffer, out region, out resources);

                for (int i = start; i < end; ++i)
                {
                    subtreeReferences.Count = 0;
                    treeletInternalNodes.Count = 0;
                    bool nodesInvalidated;
                    Tree.BinnedRefine(RefinementTargets.Elements[i], ref subtreeReferences, MaximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);
                    //Allow other refines to traverse this node.
                    Tree.nodes[RefinementTargets.Elements[i]].RefineFlag = 0;


                }


                Tree.RemoveUnusedInternalNodes(ref spareNodes);
                region.Dispose();
                Pool.GiveBack(buffer);
                spareNodes.Dispose();
                subtreeReferences.Count = 0;
                subtreeReferences.Dispose();
                treeletInternalNodes.Count = 0;
                treeletInternalNodes.Dispose();


            }

            void CacheOptimize(int workerIndex)
            {
                var startIndex = CacheOptimizeStarts.Elements[workerIndex];

                //We could wrap around. But we could also not do that because it doesn't really matter!
                var end = Math.Min(Tree.nodeCount, startIndex + CacheOptimizeCount);
                for (int i = startIndex; i < end; ++i)
                {
                    Tree.IncrementalCacheOptimizeThreadSafe(i);
                }

            }
        }

    }
}
