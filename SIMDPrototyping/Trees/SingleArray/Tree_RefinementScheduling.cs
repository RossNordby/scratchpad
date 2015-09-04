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

namespace SIMDPrototyping.Trees.SingleArray
{
    partial class Tree
    {

        unsafe float RefitAndScoreLower2(int nodeIndex, ref BoundingBox boundingBox)
        {
            var node = nodes + nodeIndex;

            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);

            var premetric = ComputeBoundsMetric(ref boundingBox);
            float changeA, changeB;
            if (node->ChildA >= 0)
            {
                changeA = RefitAndScoreLower2(node->ChildA, ref node->A);
            }
            else
                changeA = 0;
            if (node->ChildB >= 0)
            {
                changeB = RefitAndScoreLower2(node->ChildB, ref node->B);
            }
            else
                changeB = 0;
            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
            var postmetric = ComputeBoundsMetric(ref boundingBox);

            return postmetric - premetric + changeA + changeB; //TODO: would clamping produce a superior result?

        }

        unsafe float RefitAndScoreUpper2(int nodeIndex, int leafCountThreshold, ref BoundingBox boundingBox)
        {
            var node = nodes + nodeIndex;

            //All non-root nodes are guaranteed to have at least 2 children, so it's safe to access the first one.
            Debug.Assert(node->ChildCount >= 2);

            var premetric = ComputeBoundsMetric(ref boundingBox);
            float changeA, changeB;
            if (node->ChildA >= 0)
            {
                if (node->LeafCountA > leafCountThreshold)
                    changeA = RefitAndScoreUpper2(node->ChildA, leafCountThreshold, ref node->A);
                else
                    changeA = RefitAndScoreLower2(node->ChildA, ref node->A);

            }
            else
                changeA = 0;
            if (node->ChildB >= 0)
            {
                if (node->LeafCountB > leafCountThreshold)
                    changeB = RefitAndScoreUpper2(node->ChildB, leafCountThreshold, ref node->B);
                else
                    changeB = RefitAndScoreLower2(node->ChildB, ref node->B);
            }
            else
                changeB = 0;
            BoundingBox.Merge(ref node->A, ref node->B, out boundingBox);
            var postmetric = ComputeBoundsMetric(ref boundingBox);

            var childVolumeChange = changeA + changeB;

            //Cache the volume change in the node for later analysis.
            //Current volume change is not included- the choice of whether to refine a node doesn't rely on that particular node's combined bounds, only its children.
            node->LocalCostChange = postmetric >= 1e-9f ? childVolumeChange / postmetric : float.MaxValue;


            return postmetric - premetric + childVolumeChange;//TODO: would clamping produce a superior result?

        }

        unsafe float RefitAndScore2(int leafCountThreshold)
        {

            //Assumption: Index 0 is always the root if it exists, and an empty tree will have a 'root' with a child count of 0.
            if (nodes->ChildCount < 2)
            {
                Debug.Assert(nodes->ChildA < 0, "If there's only one child, it should be a leaf.");
                //If there's only a leaf (or no children), then there's no internal nodes capable of changing in volume, so there's no relevant change in cost.
                nodes->LocalCostChange = 0;
                return 0;
            }
            //TODO: Should the root node's volume change really be included? You cannot refine away the root's volume change, after all.
            var rootChildren = &nodes->ChildA;
            var rootBounds = &nodes->A;
            var rootLeafCounts = &nodes->LeafCountA;
            float childVolumeChange = 0;
            BoundingBox premerge = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            BoundingBox postmerge = premerge;
            for (int i = 0; i < nodes->ChildCount; ++i)
            {
                BoundingBox.Merge(ref rootBounds[i], ref premerge, out premerge);
                if (rootChildren[i] >= 0)
                {
#if NODE2
                    if (rootLeafCounts[i] > leafCountThreshold)
                        childVolumeChange += RefitAndScoreUpper2(rootChildren[i], leafCountThreshold, ref rootBounds[i]);
                    else
                        childVolumeChange += RefitAndScoreLower2(rootChildren[i], ref rootBounds[i]);

#else
                    if (rootLeafCounts[i] > leafCountThreshold)
                        volumeChange += RefitAndScoreUpper(rootChildren[i], leafCountThreshold, ref rootBounds[i]);
                    else
                        childVolumeChange += RefitAndScoreLower(rootChildren[i], ref rootBounds[i]);

#endif
                }
                BoundingBox.Merge(ref rootBounds[i], ref postmerge, out postmerge);
            }
            var premetric = ComputeBoundsMetric(ref premerge);
            var postmetric = ComputeBoundsMetric(ref postmerge);
            //Current volume change is not included- the choice of whether to refine a node doesn't rely on that particular node's combined bounds, only its children.
            float inversePostMetric = 1f / postmetric;
            var costChange = postmetric >= 1e-9f ? childVolumeChange * inversePostMetric : float.MaxValue;
            nodes->LocalCostChange = costChange;


            return (postmetric - premetric) * inversePostMetric + costChange;//TODO: would clamping produce a superior result?
        }


        struct MarkInvariants
        {
            public int PeriodicOffset;
            public int Period;
            public float RefinementThreshold;
            public int LeafCountThreshold;
            public float DistancePenaltySlope;
            public float DistancePenaltyOffset;
            public int MinimumDistance;
        }

        unsafe void MarkForRefinement2(int index, int distanceFromLastRefinement, ref MarkInvariants invariants, ref QuickList<int> refinementTargets)
        {
            var node = nodes + index;

            bool refined;
            if (distanceFromLastRefinement > invariants.MinimumDistance)
            {
                //Scale down the cost change of refinements which are near other refinements.
                //This reduces the frequency of pointless doublework.
                var costScale = Math.Min(1, Math.Max(0, (distanceFromLastRefinement - invariants.MinimumDistance) * invariants.DistancePenaltySlope + invariants.DistancePenaltyOffset));
                var scaledCostChange = node->LocalCostChange * costScale;

                //if (scaledCostChange > invariants.RefinementThreshold || //If true, this node's subtree changed a lot since the previous frame relative to its size.
                //    (index - invariants.PeriodicOffset) % invariants.Period == 0 || index == 0) //If true, this node was chosen as one of the periodic nodes.
                if ((index) % invariants.Period == invariants.PeriodicOffset || index == 0)
                {
                    node->RefineFlag = 1;
                    refinementTargets.Add(index);
                    refined = true;
                }
                else
                {
                    //Clear the refine flag so subtree collection properly ignores it.
                    node->RefineFlag = 0;
                    refined = false;
                }
            }
            else
            {
                node->RefineFlag = 0;
                refined = false;
            }

            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            int distance = refined ? 1 : distanceFromLastRefinement + 1;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (leafCounts[i] >= invariants.LeafCountThreshold && children[i] >= 0)
                //if (children[i] >= 0)
                {
                    MarkForRefinement2(children[i], distance, ref invariants, ref refinementTargets);
                }
            }

        }

        unsafe void MarkForRefinement22(int index, int parentLeafCount, int leafCount, int leafCountThreshold, ref QuickList<int> refinementCandidates, out int newThreshold)
        {
            Debug.Assert(leafCountThreshold > 1);

            var node = nodes + index;

            node->RefineFlag = 0; //clear out old flag info... may get set to 1 later.


            var children = &node->ChildA;
            var leafCounts = &node->LeafCountA;
            bool isWavefrontParent = false;
            newThreshold = int.MaxValue;
            for (int i = 0; i < node->ChildCount; ++i)
            {

                if (children[i] >= 0)
                {
                    if (leafCounts[i] <= leafCountThreshold)
                    {
                        //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
                        //Since we don't traverse into these children, there is no need to check the parent's leaf count.
                        refinementCandidates.Add(children[i]);
                        isWavefrontParent = true;
                        newThreshold = leafCountThreshold;
                    }
                    else
                    {
                        int candidateNewThreshold;
                        MarkForRefinement22(children[i], leafCount, leafCounts[i], leafCountThreshold, ref refinementCandidates, out candidateNewThreshold);

                        if (candidateNewThreshold < newThreshold)
                            newThreshold = candidateNewThreshold;
                    }
                }
                else
                {
                    newThreshold = leafCountThreshold;
                }
                //Leaves are not considered members of the wavefront. They're not *refinement candidates* since they're not internal nodes.
                //The threshold does not need to be modified when a leaf node is encountered.


            }

            //TODO: smaller than leafCountThreshold multiplicative factor?
            const float scale = 0.25f;
            const float minimum = 1f / (scale * scale);
            if (isWavefrontParent)
            {
                //This was a wavefront parent, so set the threshold for the parent to be accepted.
                var scaledLeafThreshold = scale * leafCountThreshold;
                newThreshold = (int)Math.Min(Math.Max(minimum, scaledLeafThreshold * scaledLeafThreshold), this.leafCount);
            }

            //if (true)//leafCount <= newThreshold && parentLeafCount > newThreshold)
            //{
            //    //This is a transition node.
            //    refinementCandidates.Add(index);
            //    newThreshold = Math.Min(newThreshold * (int)(leafCountThreshold * scale), this.leafCount);
            //}






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
            int maximumSubtrees = (int)(Math.Sqrt(leafCount) * 2);
            var refinementTargets = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex((int)(leafCount / (maximumSubtrees * 0.5f))));

            int leafCountThreshold = Math.Min(leafCount, maximumSubtrees);

            //ValidateRefineFlags(0);
            var costChange = RefitAndScore2(leafCountThreshold);

            //Collect the refinement candidates.
            int newThreshold;
            MarkForRefinement22(0, int.MaxValue, this.leafCount, leafCountThreshold, ref refinementTargets, out newThreshold);
            //ValidateRefineFlags(0);

            var refineAggressiveness = Math.Max(0, costChange * refineAggressivenessScale);
            float refinePortion = Math.Min(1, refineAggressiveness * 0.5f);
            var targetRefinementScale = Math.Max(Math.Ceiling(refinementTargets.Count * 0.03f), refinementTargets.Count * refinePortion);
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
                nodes->RefineFlag = 1;
            }
            //Console.WriteLine($"Refinement count: {refinementTargets.Count}");


            //if (costChange > 1000)
            //{
            //    //Something silly is going on. The usual approach does not scale well to this level of explosive corruption.
            //    //Just rebuild it.
            //    maximumSubtrees = LeafCount;
            //    refinementTargets.Add(0);
            //    //Make sure collectsubtrees can gather all the nodes.
            //    for (int i = 0; i < refinementTargets.Count; ++i)
            //    {
            //        nodes[i].RefineFlag = 0;
            //    }
            //    //Note: yes, this does some redundant work. But the amount of work required to do the partially redundant MarkForRefinement is absolutely nothing compared to this full rebuild.

            //}

            //Refine all marked targets.
            var pool = BufferPools<int>.Thread;

            var spareNodes = new QuickList<int>(pool, 8);
            var subtreeReferences = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            var treeletInternalNodes = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            int[] buffer;
            MemoryRegion region;
            BinnedResources resources;
            CreateBinnedResources(pool, maximumSubtrees, out buffer, out region, out resources);


            //var visitedNodes = new QuickSet<int>(BufferPools<int>.Thread, BufferPools<int>.Thread);
            //int numberOfDuplicates = 0;
            //for (int i = refinementTargets.Count - 1; i >= 0; --i)
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
            //for (int i = 0; i < refinementTargets.Count; ++i)
            //{
            //    nodes[refinementTargets.Elements[i]].RefineFlag = 0;
            //}
            //Console.WriteLine($"Fraction of internal nodes visited: {visitedNodes.Count / (double)NodeCount}");
            //Console.WriteLine($"Fraction of duplicates visited: {(visitedNodes.Count > 0 ? (numberOfDuplicates / (double)visitedNodes.Count) : 0)}");
            //visitedNodes.Dispose();

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

            //It's safe to use the refinementTargets' nodes because the refinements never move the treelet roots.
            //(Note: if you moved the RefineFlag reset into the loop, this guarantee goes out the window because the root refine could destroy the guarantee.)
            //for (int i = 0; i < refinementTargets.Count - 1; ++i) //Exclude the root. It's always the last target.
            //{
            //    RecursiveIncrementalCacheOptimizeLocking(refinementTargets.Elements[i]);

            //    int foundNodes, scorableNodeCount;
            //    float nodeScore;
            //    MeasureCacheQuality(refinementTargets.Elements[i], out foundNodes, out nodeScore, out scorableNodeCount);
            //    Console.WriteLine($"CACHE QUALITY {refinementTargets[i]}: {MeasureCacheQuality(refinementTargets.Elements[i])}, NODE COUNT: {foundNodes}");
            //}

            //RecursiveIncrementalCacheOptimizeLocking(0, 15);

            //To multithread this, give each worker a contiguous chunk of nodes. You want to do the biggest chunks possible to chain decent cache behavior as far as possible.
            var cacheOptimizeAggressiveness = Math.Max(0, costChange * cacheOptimizeAggressivenessScale);
            float cacheOptimizePortion = Math.Max(0.01f, Math.Min(1, cacheOptimizeAggressiveness * 0.75f));
            var cacheOptimizeCount = (int)Math.Ceiling(cacheOptimizePortion * nodeCount);

            var startIndex = (int)(((long)frameIndex * cacheOptimizeCount) % nodeCount);

            //We could wrap around. But we could also not do that because it doesn't really matter!
            var end = Math.Min(NodeCount, startIndex + cacheOptimizeCount);
            for (int i = startIndex; i < end; ++i)
            {

                IncrementalCacheOptimizeLocking(i);
                //tree.IncrementalCacheOptimize(i);
            }
            //Console.WriteLine($"Cache optimize count: {cacheOptimizeCount}, effective: {end - startIndex}");

            return actualRefinementTargetsCount;
        }




    }
}
