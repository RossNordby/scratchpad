﻿using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {

        unsafe float RefitAndMeasure(int nodeIndex, ref BoundingBox boundingBox)
        {
            var node = nodes + nodeIndex;

            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first ones.
            Debug.Assert(node->ChildCount >= 2);

            var premetric = ComputeBoundsMetric(ref boundingBox);
            float childChange = 0;
            if (node->ChildA >= 0)
            {
                childChange += RefitAndMeasure(node->ChildA, ref node->A);
            }
            if (node->ChildB >= 0)
            {
                childChange += RefitAndMeasure(node->ChildB, ref node->B);
            }
            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);

#if !NODE2
            for (int i = 2; i < node->ChildCount; ++i)
            {
                var child = (&node->ChildA)[i];
                if (child >= 0)
                {
                    childChange = RefitAndScoreLower2(child, ref (&node->A)[i]);
                }
                BoundingBox.Merge(ref boundingBox, ref (&node->A)[i], out boundingBox);
            }
#endif

            var postmetric = ComputeBoundsMetric(ref boundingBox);
            return postmetric - premetric + childChange; //TODO: would clamping produce a superior result?

        }

        unsafe float RefitAndMark(int index, int leafCountThreshold, ref QuickList<int> refinementCandidates, ref BoundingBox boundingBox)
        {
            Debug.Assert(leafCountThreshold > 1);

            var node = nodes + index;
            Debug.Assert(node->ChildCount >= 2);
            Debug.Assert(node->RefineFlag == 0);
            float childChange = 0;

            var premetric = ComputeBoundsMetric(ref boundingBox);
            //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
            //Add them to a list of refinement candidates.
            //Note that leaves are not included, since they can't be refinement candidates.
            if (node->ChildA >= 0)
            {
                if (node->LeafCountA <= leafCountThreshold)
                {
                    refinementCandidates.Add(node->ChildA);
                    childChange += RefitAndMeasure(node->ChildA, ref node->A);
                }
                else
                {
                    childChange += RefitAndMark(node->ChildA, leafCountThreshold, ref refinementCandidates, ref node->A);
                }
            }
            if (node->ChildB >= 0)
            {
                if (node->LeafCountB <= leafCountThreshold)
                {
                    refinementCandidates.Add(node->ChildB);
                    childChange += RefitAndMeasure(node->ChildB, ref node->B);
                }
                else
                {
                    childChange += RefitAndMark(node->ChildB, leafCountThreshold, ref refinementCandidates, ref node->B);
                }
            }

            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);

#if !NODE2
            var bounds = &node->A;
            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    if (leafCounts[i] <= leafCountThreshold)
                    {
                        //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
                        //Since we don't traverse into these children, there is no need to check the parent's leaf count.
                        refinementCandidates.Add(children[i]);
                        childChange += RefitAndMeasure(children[i], ref bounds[i]);
                    }
                    else
                    {
                        childChange += RefitAndMark(children[i], leafCountThreshold, ref refinementCandidates, ref bounds[i]);
                    }
                }
                BoundingBox.Merge(ref bounds[i], ref boundingBox, out boundingBox);
                //Leaves are not considered members of the wavefront. They're not *refinement candidates* since they're not internal nodes.
            }
#endif
            var postmetric = ComputeBoundsMetric(ref boundingBox);

            return postmetric - premetric + childChange; //TODO: Would clamp provide better results?



        }

        unsafe float RefitAndMark(int leafCountThreshold, ref QuickList<int> refinementCandidates)
        {
            if (nodes->ChildCount < 2)
            {
                Debug.Assert(nodes->ChildA < 0, "If there's only one child, it should be a leaf.");
                //If there's only a leaf (or no children), then there's no internal nodes capable of changing in volume, so there's no relevant change in cost.
                return 0;
            }

            var bounds = &nodes->A;
            var children = &nodes->ChildA;
            var leafCounts = &nodes->LeafCountA;
            float childChange = 0;
            BoundingBox premerge = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            BoundingBox postmerge = premerge;
            for (int i = 0; i < nodes->ChildCount; ++i)
            {
                BoundingBox.Merge(ref bounds[i], ref premerge, out premerge);
                //Note: these conditions mean the root will never be considered a wavefront node. That's acceptable;
                //it will be included regardless.
                if (children[i] >= 0)
                {
                    if (leafCounts[i] <= leafCountThreshold)
                    {
                        //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
                        //Since we don't traverse into these children, there is no need to check the parent's leaf count.
                        refinementCandidates.Add(children[i]);
                        childChange += RefitAndMeasure(children[i], ref bounds[i]);
                    }
                    else
                    {
                        childChange += RefitAndMark(children[i], leafCountThreshold, ref refinementCandidates, ref bounds[i]);
                    }
                }
                BoundingBox.Merge(ref bounds[i], ref postmerge, out postmerge);
            }

            var premetric = ComputeBoundsMetric(ref premerge);
            var postmetric = ComputeBoundsMetric(ref postmerge);

            if (postmetric >= 0)
            {
                return (postmetric - premetric + childChange) / postmetric;
            }
            return 0;
        }

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

            public QuickList<int> RefitNodes;
            public float RefitVolumeChange;
            public int LeafCountThreshold;
            public Action<int> RefitAndMarkAction;

            public QuickList<int> RefinementTargets;
            public QuickList<int> RefinementStarts;
            public BufferPool<int> Pool;
            public int MaximumSubtrees;
            public Action<int> RefineAction;

            QuickList<int> CacheOptimizeStarts;
            public int CacheOptimizeCount;
            public Action<int> CacheOptimizeAction;

            public RefitAndRefineMultithreadedContext(Tree tree)
            {
                Tree = tree;
                RefitAndMarkAction = RefitAndMark;
                RefineAction = Refine;
                CacheOptimizeAction = CacheOptimize;
            }

            unsafe void RefitAndMark(int refitNodeIndex)
            {
                var node = Tree.nodes + RefitNodes.Elements[refitNodeIndex];
                Debug.Assert(node->Parent >= 0, "The root should not be marked for refit.");
                var parent = Tree.nodes + node->Parent;
                var boundingBoxInParent = &parent->A + node->IndexInParent;
                node->LocalCostChange = Tree.RefitAndMark(RefitNodes.Elements[refitNodeIndex], LeafCountThreshold, ref RefinementTargets, ref *boundingBoxInParent);

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



        unsafe void ValidateRefineFlags(int index)
        {
            var node = nodes + index;
            if (node->RefineFlag != 0)
                Console.WriteLine("BAD");

            var children = &node->ChildA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                {
                    ValidateRefineFlags(children[i]);
                }
            }
        }

        public unsafe int RefitAndRefine(int frameIndex, float refineAggressivenessScale = 1, float cacheOptimizeAggressivenessScale = 1)
        {
            //Don't proceed if the tree is empty.
            if (leafCount == 0)
                return 0;
            int maximumSubtrees = (int)(Math.Sqrt(leafCount) * 3);
            var pool = BufferPools<int>.Locking;
            var refinementTargets = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex((int)(leafCount / (maximumSubtrees * 0.5f))));

            int leafCountThreshold = Math.Min(leafCount, maximumSubtrees);

            //Collect the refinement candidates.
            var costChange = RefitAndMark(leafCountThreshold, ref refinementTargets);

            var refineAggressiveness = Math.Max(0, costChange * refineAggressivenessScale);
            float refinePortion = Math.Min(1, refineAggressiveness * 0.25f);
            var targetRefinementScale = Math.Max(2, (float)Math.Ceiling(refinementTargets.Count * 0.03f)) + refinementTargets.Count * refinePortion;
            var period = (int)(refinementTargets.Count / targetRefinementScale);
            var offset = (int)((frameIndex * 236887691L + 104395303L) % refinementTargets.Count);


            int actualRefinementTargetsCount = 0;
            int targetRefinementCount = (int)targetRefinementScale;
            for (int i = 0; i < targetRefinementCount - 1; ++i)
            {
                var index = i * period + offset;
                if (index >= refinementTargets.Count)
                    index -= refinementTargets.Count;
                Debug.Assert(index < refinementTargets.Count && index >= 0);
                refinementTargets[actualRefinementTargetsCount++] = refinementTargets[index];
                nodes[refinementTargets[index]].RefineFlag = 1;
            }
            refinementTargets.Count = actualRefinementTargetsCount;
            if (nodes->RefineFlag != 1)
            {
                refinementTargets.Add(0);
                ++actualRefinementTargetsCount;
                nodes->RefineFlag = 1;
            }


            //Refine all marked targets.

            var spareNodes = new QuickList<int>(pool, 8);
            var subtreeReferences = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            var treeletInternalNodes = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            int[] buffer;
            MemoryRegion region;
            BinnedResources resources;
            CreateBinnedResources(pool, maximumSubtrees, out buffer, out region, out resources);

            for (int i = 0; i < refinementTargets.Count; ++i)
            {
                subtreeReferences.Count = 0;
                treeletInternalNodes.Count = 0;
                bool nodesInvalidated;
                BinnedRefine(refinementTargets.Elements[i], ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);
                //TODO: Should this be moved into a post-loop? It could permit some double work, but that's not terrible.
                //It's not invalid from a multithreading perspective, either- setting the refine flag to zero is essentially an unlock.
                //If other threads don't see it updated due to cache issues, it doesn't really matter- it's not a signal or anything like that.
                nodes[refinementTargets.Elements[i]].RefineFlag = 0;


            }


            RemoveUnusedInternalNodes(ref spareNodes);
            region.Dispose();
            pool.GiveBack(buffer);
            spareNodes.Dispose();
            subtreeReferences.Count = 0;
            subtreeReferences.Dispose();
            treeletInternalNodes.Count = 0;
            treeletInternalNodes.Dispose();
            refinementTargets.Count = 0;
            refinementTargets.Dispose();


            //To multithread this, give each worker a contiguous chunk of nodes. You want to do the biggest chunks possible to chain decent cache behavior as far as possible.
            var cacheOptimizeAggressiveness = Math.Max(0, costChange * cacheOptimizeAggressivenessScale);
            float cacheOptimizePortion = Math.Min(1, 0.02f + cacheOptimizeAggressiveness * 0.75f);
            var cacheOptimizeCount = (int)Math.Ceiling(cacheOptimizePortion * nodeCount);

            var startIndex = (int)(((long)frameIndex * cacheOptimizeCount) % nodeCount);

            //We could wrap around. But we could also not do that because it doesn't really matter!
            var end = Math.Min(NodeCount, startIndex + cacheOptimizeCount);
            for (int i = startIndex; i < end; ++i)
            {
                IncrementalCacheOptimize(i);
            }

            return actualRefinementTargetsCount;
        }




    }
}
