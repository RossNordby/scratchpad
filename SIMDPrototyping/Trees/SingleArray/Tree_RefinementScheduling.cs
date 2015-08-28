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
            float childVolumeChange = 0;
            BoundingBox premerge = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            BoundingBox postmerge = premerge;
            for (int i = 0; i < nodes->ChildCount; ++i)
            {
                BoundingBox.Merge(ref rootBounds[i], ref premerge, out premerge);
                if (rootChildren[i] >= 0)
                {
#if NODE2
                    childVolumeChange += RefitAndScoreUpper2(rootChildren[i], leafCountThreshold, ref rootBounds[i]);
#else
                    volumeChange += RefitAndScoreUpper(rootChildren[i], leafCountThreshold, ref rootBounds[i]);
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

        public unsafe void RefitAndRefine(ref QuickList<int> refinementTargets, int frameIndex, float aggressivenessScale, int maximumSubtrees = 1024)
        {
            var levelsInRefine = Math.Log(maximumSubtrees, ChildrenCapacity);

            MarkInvariants invariants;
            invariants.LeafCountThreshold = (int)Math.Min(leafCount, 0.75f * maximumSubtrees);

            var costChange = RefitAndScore2(invariants.LeafCountThreshold);
            var aggressiveness = Math.Max(0, costChange * aggressivenessScale);

            //Higher aggressiveness->lower period.
            //Is the need for higher aggressiveness linear, or nonlinear? Nonlinear unbounded seems more obvious.
            invariants.Period = (int)(16f / (aggressiveness + 1)) + 1;
            //invariants.PeriodicOffset = (int)((frameIndex * 236887691L + 104395303L) % invariants.Period);
            invariants.PeriodicOffset = (int)((frameIndex * 79151L) % invariants.Period);
            invariants.MinimumDistance = (int)(levelsInRefine * 0.5);
            const float minimumMultiplier = 0.1f;
            invariants.DistancePenaltyOffset = minimumMultiplier;
            invariants.DistancePenaltySlope = (float)((1 - minimumMultiplier) / (levelsInRefine * 0.65f));
            invariants.RefinementThreshold = 1f / (aggressiveness + 1);

            MarkForRefinement2(0, (int)(levelsInRefine), ref invariants, ref refinementTargets);

            //Refine all marked targets.
            var pool = BufferPools<int>.Thread;

            var spareNodes = new QuickList<int>(pool, 8);
            var subtreeReferences = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            var treeletInternalNodes = new QuickQueue<int>(pool, BufferPool<int>.GetPoolIndex(maximumSubtrees));
            int[] buffer;
            MemoryRegion region;
            BinnedResources resources;
            CreateBinnedResources(pool, maximumSubtrees, out buffer, out region, out resources);

            for (int i = 0; i < refinementTargets.Count; ++i)
            {
                subtreeReferences.Count = 0;
                treeletInternalNodes.FastClear();
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
            subtreeReferences.Dispose();
            treeletInternalNodes.Dispose();
        }




    }
}
