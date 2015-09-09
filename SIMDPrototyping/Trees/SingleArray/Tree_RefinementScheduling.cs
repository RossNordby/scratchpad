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
            var refinementTargets = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex((int)((leafCount * ChildrenCapacity) / (maximumSubtrees))));

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
