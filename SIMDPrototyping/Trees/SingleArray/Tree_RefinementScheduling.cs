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
            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            for (int i = 0; i < nodes->ChildCount; ++i)
            {
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
                BoundingBox.Merge(ref bounds[i], ref merged, out merged);
            }

            var postmetric = ComputeBoundsMetric(ref merged);

            //Note that the root's own change is not included.
            //This cost change is used to determine whether or not to refine.
            //Since refines are unable to change the volume of the root, there's
            //no point in including it in the volume change.
            //It does, however, normalize the child volume changes into a cost metric.
            if (postmetric >= 0)
            {
                return childChange / postmetric;
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

        void GetRefitAndMarkTuning(out int maximumSubtrees, out int estimatedRefinementCandidateCount, out int leafCountThreshold)
        {
            maximumSubtrees = (int)(Math.Sqrt(leafCount) * 3);
            estimatedRefinementCandidateCount = (leafCount * ChildrenCapacity) / maximumSubtrees;

            leafCountThreshold = Math.Min(leafCount, maximumSubtrees);
        }


        void GetRefineTuning(int frameIndex, int refinementCandidatesCount, float refineAggressivenessScale, float costChange,
            out int targetRefinementCount, out int refinementPeriod, out int refinementOffset)
        {
            var refineAggressiveness = Math.Max(0, costChange * refineAggressivenessScale);
            float refinePortion = Math.Min(1, refineAggressiveness * 0.25f);

            var targetRefinementScale = Math.Max(2, (float)Math.Ceiling(refinementCandidatesCount * 0.03f)) + refinementCandidatesCount * refinePortion;
            refinementPeriod = (int)(refinementCandidatesCount / targetRefinementScale);
            refinementOffset = (int)((frameIndex * 236887691L + 104395303L) % refinementCandidatesCount);

            targetRefinementCount = (int)targetRefinementScale;
        }

        public int GetCacheOptimizeTuning(float costChange, float cacheOptimizeAggressivenessScale)
        {
            var cacheOptimizeAggressiveness = Math.Max(0, costChange * cacheOptimizeAggressivenessScale);
            float cacheOptimizePortion = Math.Min(1, 0.03f + cacheOptimizeAggressiveness * 0.5f);
            return (int)Math.Ceiling(cacheOptimizePortion * nodeCount);
        }


        unsafe void ValidateOptimalPositions(ref QuickList<int> optimalPositions)
        {
            int nodeCount = 0;
            ValidateOptimalPositions(0, ref optimalPositions, ref nodeCount);
        }
        unsafe void ValidateOptimalPositions(int nodeIndex, ref QuickList<int> optimalPositions, ref int nodeCount)
        {
            var node = nodes + nodeIndex;
            if (node->RefineFlag > 0)
            {
                optimalPositions[node->RefineFlag - 1] = nodeCount;
                //if (optimalPositions[node->RefineFlag - 1] != nodeCount)
                //    Console.WriteLine("BAD");
            }
            ++nodeCount;
            var children = &node->ChildA;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                if (children[i] >= 0)
                    ValidateOptimalPositions(children[i], ref optimalPositions, ref nodeCount);
            }
        }

        public unsafe int RefitAndRefine(int frameIndex, float refineAggressivenessScale = 1, float cacheOptimizeAggressivenessScale = 1)
        {
            //Don't proceed if the tree is empty.
            if (leafCount == 0)
                return 0;
            var pool = BufferPools<int>.Locking;
            int maximumSubtrees, estimatedRefinementCandidateCount, leafCountThreshold;
            GetRefitAndMarkTuning(out maximumSubtrees, out estimatedRefinementCandidateCount, out leafCountThreshold);

            var refinementCandidates = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(estimatedRefinementCandidateCount));

            //Collect the refinement candidates.
            var costChange = RefitAndMark(leafCountThreshold, ref refinementCandidates);


            int targetRefinementCount, period, offset;
            GetRefineTuning(frameIndex, refinementCandidates.Count, refineAggressivenessScale, costChange, out targetRefinementCount, out period, out offset);


            var refinementTargets = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(targetRefinementCount));

            int actualRefinementTargetsCount = 0;
            int index = offset;
            for (int i = 0; i < targetRefinementCount - 1; ++i)
            {
                index += period;
                if (index >= refinementCandidates.Count)
                    index -= refinementCandidates.Count;
                Debug.Assert(index < refinementCandidates.Count && index >= 0);

                refinementTargets.Elements[actualRefinementTargetsCount++] = refinementCandidates.Elements[index];
                nodes[refinementCandidates.Elements[index]].RefineFlag = i + 1;
            }
            refinementTargets.Count = actualRefinementTargetsCount;
            refinementCandidates.Count = 0;
            refinementCandidates.Dispose();
            if (nodes->RefineFlag == 0)
            {
                refinementTargets.Add(0);
                nodes->RefineFlag = refinementTargets.Count;
                ++actualRefinementTargetsCount;
            }


            //TODO: WHen you delete these, you can undo the i/count setting of refineflags.
            //var optimalTargetPositions = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(targetRefinementCount));
            //for (int refinementTargetIndex = 0; refinementTargetIndex < refinementTargets.Count; ++refinementTargetIndex)
            //{
            //    var node = nodes + refinementTargets.Elements[refinementTargetIndex];
            //    int optimumIndex = 0;
            //    while (node->Parent >= 0)
            //    {
            //        //Each node on the path to the root must be counted.
            //        //++optimumIndex;
            //        var childIndex = node->IndexInParent;
            //        node = nodes + node->Parent;
            //        var children = &node->ChildA;
            //        var leafCounts = &node->LeafCountA;
            //        for (int i = 0; i < childIndex; ++i)
            //        {
            //            //The number of nodes in a binary tree needed to support n leaves is n - 1.
            //            //So, the index of the node immediately following a subtree of n nodes is exactly n.
            //            optimumIndex += leafCounts[i];
            //        }
            //    }
            //    optimalTargetPositions.Add(optimumIndex);

            //}

            //ValidateOptimalPositions(ref optimalTargetPositions);

            ////Swap all refinement targets into their global optimum locations.

            //for (int i = 0; i < refinementTargets.Count; ++i)
            //{
            //    if (refinementTargets.Elements[i] != optimalTargetPositions.Elements[i])
            //    {
            //        //Need to perform a swap.
            //        //Check to see if the swap target is also a refinement target.
            //        var refineIndex = nodes[optimalTargetPositions.Elements[i]].RefineFlag - 1;
            //        if (refineIndex >= 0)
            //        {
            //            //The swap target is in the refinement target set. This should be very rare, but it must be handled carefully.
            //            //The swap target will come to inhabit the current refinement target's position.
            //            refinementTargets.Elements[refineIndex] = refinementTargets.Elements[i];
            //        }
            //        SwapNodes(refinementTargets.Elements[i], optimalTargetPositions.Elements[i]);
            //        //Update the targets list.
            //        //TODO: could avoid this set by just using the optimal target positions at the end instead.
            //        refinementTargets.Elements[i] = optimalTargetPositions.Elements[i];
            //    }
            //}
            //for (int i = 0; i < refinementTargets.Count; ++i)
            //{
            //    if (nodes[optimalTargetPositions[i]].RefineFlag != i + 1)
            //    {
            //        Console.WriteLine("bad");
            //    }
            //}
            //optimalTargetPositions.Count = 0;
            //optimalTargetPositions.Dispose();

            ////Cache optimize the complete subtrees of all non-root refinement targets.
            //for (int i = 0; i < refinementTargets.Count; ++i)
            //{
            //    if (refinementTargets.Elements[i] != 0)
            //    {
            //        CacheOptimize(refinementTargets.Elements[i]);
            //    }
            //}

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

            var cacheOptimizeCount = GetCacheOptimizeTuning(costChange, cacheOptimizeAggressivenessScale);

            var startIndex = (int)(((long)frameIndex * cacheOptimizeCount) % nodeCount);

            //We could wrap around. But we could also not do that because it doesn't really matter!
            var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            var end = Math.Min(NodeCount, startIndex + cacheOptimizeCount);
            for (int i = startIndex; i < end; ++i)
            {
                IncrementalCacheOptimize(i);
            }
            var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            Console.WriteLine($"Cache optimize time: {endTime - startTime}");

            return actualRefinementTargetsCount;
        }




    }
}
