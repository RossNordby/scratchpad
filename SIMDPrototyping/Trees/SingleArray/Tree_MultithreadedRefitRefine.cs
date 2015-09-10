using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using BEPUutilities.Threading;
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



        unsafe void CollectNodesForMultithreadedRefit(int nodeIndex,
            int multithreadingLeafCountThreshold, ref QuickList<int> refitAndMarkTargets,
            int refinementLeafCountThreshold, ref QuickList<int> refinementCandidates)
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
                    if (leafCounts[i] <= multithreadingLeafCountThreshold)
                    {
                        if (leafCounts[i] <= refinementLeafCountThreshold)
                        {
                            //It's possible that a wavefront node is this high in the tree, so it has to be captured here because the postpass won't find it.
                            refinementCandidates.Add(children[i]);
                        }
                        refitAndMarkTargets.Add(children[i]);
                    }
                    else
                    {
                        CollectNodesForMultithreadedRefit(children[i], multithreadingLeafCountThreshold, ref refitAndMarkTargets, refinementLeafCountThreshold, ref refinementCandidates);
                    }
                }
            }
        }


        void CollectNodesForMultithreadedRefit(int threadCount, ref QuickList<int> targets,
            int refinementLeafCountThreshold, ref QuickList<int> refinementCandidates)
        {
            //No point in using this if there aren't enough leaves.
            Debug.Assert(leafCount > 2);
            int multithreadingLeafCountThreshold = leafCount / (threadCount * 2);
            if (multithreadingLeafCountThreshold < refinementLeafCountThreshold)
                multithreadingLeafCountThreshold = refinementLeafCountThreshold;
            CollectNodesForMultithreadedRefit(0, multithreadingLeafCountThreshold, ref targets, refinementLeafCountThreshold, ref refinementCandidates);
        }

        /// <summary>
        /// Caches input and output for the multithreaded execution of a tree's refit and refinement operations.
        /// </summary>
        public class RefitAndRefineMultithreadedContext
        {
            public Tree Tree;

            public int RefitNodeIndex;
            public QuickList<int> RefitNodes;
            public float RefitCostChange;
            public int LeafCountThreshold;
            public RawList<QuickList<int>> RefinementCandidates;
            public Action<int> RefitAndMarkAction;

            public int RefineIndex;
            public QuickList<int> RefinementTargets;
            public BufferPool<int> Pool;
            public int MaximumSubtrees;
            public Action<int> RefineAction;

            public QuickList<int> CacheOptimizeStarts;
            public int PerWorkerCacheOptimizeCount;
            public Action<int> CacheOptimizeAction;

            public RefitAndRefineMultithreadedContext(Tree tree)
            {
                Tree = tree;
                RefitAndMarkAction = RefitAndMark;
                RefineAction = Refine;
                CacheOptimizeAction = CacheOptimize;
                RefinementCandidates = new RawList<QuickList<int>>(Environment.ProcessorCount);
            }

            public void Initialize(int workerCount, int estimatedRefinementCandidates, BufferPool<int> pool)
            {
                RefitNodeIndex = -1;

                RefitNodes = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(workerCount));

                if (RefinementCandidates.Capacity < workerCount)
                    RefinementCandidates.Capacity = workerCount;
                RefinementCandidates.Count = workerCount;
                var perThreadPoolIndex = BufferPool<int>.GetPoolIndex((int)Math.Ceiling(estimatedRefinementCandidates / (float)workerCount));
                for (int i = 0; i < workerCount; ++i)
                {
                    RefinementCandidates.Elements[i] = new QuickList<int>(pool, perThreadPoolIndex);
                }

                RefineIndex = -1;
                //Note that refinement targets are NOT initialized here, because we don't know the size yet.

                CacheOptimizeStarts = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(workerCount));
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

                RefinementTargets.Count = 0;
                RefinementTargets.Dispose();

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


                    //int foundLeafCount;
                    //Tree.Validate(RefitNodes.Elements[refitNodeIndex], node->Parent, node->IndexInParent, ref *boundingBoxInParent, out foundLeafCount);


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
                                //We do, however, need to divide by root volume so that we get the change in cost metric rather than volume.
                                var merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
                                var bounds = &node->A;
                                for (int i = 0; i < node->ChildCount; ++i)
                                {
                                    BoundingBox.Merge(ref bounds[i], ref merged, out merged);
                                }
                                var postmetric = ComputeBoundsMetric(ref merged);
                                if (postmetric > 1e-9f)
                                    RefitCostChange = node->LocalCostChange / postmetric;
                                else
                                    RefitCostChange = 0;
                                //Clear the root's refine flag (unioned).
                                node->RefineFlag = 0;
                                break;
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
                            break;
                        }
                    }

                }


            }

            unsafe void Refine(int workerIndex)
            {
                var spareNodes = new QuickList<int>(Pool, 8);
                var subtreeReferences = new QuickList<int>(Pool, BufferPool<int>.GetPoolIndex(MaximumSubtrees));
                var treeletInternalNodes = new QuickList<int>(Pool, BufferPool<int>.GetPoolIndex(MaximumSubtrees));
                int[] buffer;
                MemoryRegion region;
                BinnedResources resources;
                CreateBinnedResources(Pool, MaximumSubtrees, out buffer, out region, out resources);

                int refineIndex;
                while ((refineIndex = Interlocked.Increment(ref RefineIndex)) < RefinementTargets.Count)
                {

                    subtreeReferences.Count = 0;
                    treeletInternalNodes.Count = 0;
                    bool nodesInvalidated;
                    Tree.BinnedRefine(RefinementTargets.Elements[refineIndex], ref subtreeReferences, MaximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);
                    //Allow other refines to traverse this node.
                    Tree.nodes[RefinementTargets.Elements[refineIndex]].RefineFlag = 0;
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
                var end = Math.Min(Tree.nodeCount, startIndex + PerWorkerCacheOptimizeCount);
                for (int i = startIndex; i < end; ++i)
                {
                    Tree.IncrementalCacheOptimizeThreadSafe(i);
                }

            }
        }



        unsafe void CheckForRefinementOverlaps(int nodeIndex, ref QuickList<int> refinementTargets)
        {

            var node = nodes + nodeIndex;
            var children = &node->ChildA;
            for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
            {
                if (children[childIndex] >= 0)
                {
                    for (int i = 0; i < refinementTargets.Count; ++i)
                    {
                        if (refinementTargets.Elements[i] == children[childIndex])
                            Console.WriteLine("Found a refinement target in the children of a refinement target.");
                    }

                    CheckForRefinementOverlaps(children[childIndex], ref refinementTargets);
                }

            }
        }

        public unsafe int RefitAndRefine(int frameIndex, IParallelLooper looper, RefitAndRefineMultithreadedContext context, float refineAggressivenessScale = 1, float cacheOptimizeAggressivenessScale = 1)
        {
            //Don't proceed if the tree is empty.
            if (leafCount == 0)
                return 0;
            var pool = BufferPools<int>.Locking;

            int estimatedRefinementTargetCount;
            GetRefitAndMarkTuning(out context.MaximumSubtrees, out estimatedRefinementTargetCount, out context.LeafCountThreshold);

            context.Initialize(looper.ThreadCount, estimatedRefinementTargetCount, pool);

            CollectNodesForMultithreadedRefit(looper.ThreadCount, ref context.RefitNodes, context.LeafCountThreshold, ref context.RefinementCandidates.Elements[0]);

            //Collect the refinement candidates.
            looper.ForLoop(0, looper.ThreadCount, context.RefitAndMarkAction);


            var refinementCandidatesCount = 0;
            for (int i = 0; i < looper.ThreadCount; ++i)
            {
                refinementCandidatesCount += context.RefinementCandidates.Elements[i].Count;
            }

            int targetRefinementCount, period, offset;
            GetRefineTuning(frameIndex, refinementCandidatesCount, refineAggressivenessScale, context.RefitCostChange, out targetRefinementCount, out period, out offset);


            //Condense the set of candidates into a set of targets.
            context.RefinementTargets = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(targetRefinementCount));

            int actualRefinementTargetsCount = 0;
            var currentCandidatesIndex = 0;
            int index = offset;
            for (int i = 0; i < targetRefinementCount - 1; ++i)
            {
                index += period;
                //Wrap around if the index doesn't fit.
                while (index >= context.RefinementCandidates.Elements[currentCandidatesIndex].Count)
                {
                    index -= context.RefinementCandidates.Elements[currentCandidatesIndex].Count;
                    ++currentCandidatesIndex;
                    if (currentCandidatesIndex >= context.RefinementCandidates.Count)
                        currentCandidatesIndex -= context.RefinementCandidates.Count;
                }
                Debug.Assert(index < context.RefinementCandidates.Elements[currentCandidatesIndex].Count && index >= 0);
                var nodeIndex = context.RefinementCandidates.Elements[currentCandidatesIndex].Elements[index];
                context.RefinementTargets.Elements[actualRefinementTargetsCount++] = nodeIndex;
                nodes[nodeIndex].RefineFlag = 1;
            }
            context.RefinementTargets.Count = actualRefinementTargetsCount;
            if (nodes->RefineFlag != 1)
            {
                context.RefinementTargets.Add(0);
                ++actualRefinementTargetsCount;
                nodes->RefineFlag = 1;
            }

            //int actualRefinementTargetsCount = 2;
            //context.RefinementTargets.Add(nodes->ChildA);
            //context.RefinementTargets.Add(nodes->ChildB);
            //nodes[nodes->ChildA].RefineFlag = 1;
            //nodes[nodes->ChildB].RefineFlag = 1;

            Console.Write("Refinement nodes: ");
            for (int i = 0; i < context.RefinementTargets.Count; ++i)
            {
                Console.Write($"{context.RefinementTargets[i]}, ");
            }
            Console.WriteLine();

            for (int i = 0; i < context.RefinementTargets.Count - 1; ++i)
            {
                CheckForRefinementOverlaps(context.RefinementTargets.Elements[i], ref context.RefinementTargets);
            }

            //Refine all marked targets.
            looper.ForLoop(0, Math.Min(looper.ThreadCount, context.RefinementTargets.Count), context.RefineAction);



            ////To multithread this, give each worker a contiguous chunk of nodes. You want to do the biggest chunks possible to chain decent cache behavior as far as possible.
            //var cacheOptimizeCount = GetCacheOptimizeTuning(context.RefitCostChange, cacheOptimizeAggressivenessScale);

            //context.PerWorkerCacheOptimizeCount = cacheOptimizeCount / looper.ThreadCount;
            //var startIndex = (int)(((long)frameIndex * context.PerWorkerCacheOptimizeCount) % nodeCount);
            //context.CacheOptimizeStarts.Add(startIndex);

            //var optimizationSpacing = nodeCount / looper.ThreadCount;
            //var optimizationSpacingWithExtra = optimizationSpacing + 1;
            //var optimizationRemainder = nodeCount - optimizationSpacing * looper.ThreadCount;

            //for (int i = 1; i < looper.ThreadCount; ++i)
            //{
            //    if (optimizationRemainder > 0)
            //    {
            //        startIndex += optimizationSpacingWithExtra;
            //        --optimizationRemainder;
            //    }
            //    else
            //    {
            //        startIndex += optimizationSpacing;
            //    }
            //    if (startIndex > NodeCount)
            //        startIndex -= nodeCount;
            //    Debug.Assert(startIndex >= 0 && startIndex < nodeCount);
            //    context.CacheOptimizeStarts.Add(startIndex);
            //}

            //looper.ForLoop(0, looper.ThreadCount, context.CacheOptimizeAction);

            context.CleanUp();
            return actualRefinementTargetsCount;
        }


    }
}
