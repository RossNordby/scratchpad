using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {

        unsafe float RefitAndMeasure(ref NodeChild child)
        {
            var node = nodes + child.Index;

            //All nodes are guaranteed to have at least 2 children.
            Debug.Assert(leafCount >= 2);

            var premetric = ComputeBoundsMetric(ref child.Min, ref child.Max);
            float childChange = 0;
            ref var a = ref node->A;
            if (a.Index >= 0)
            {
                childChange += RefitAndMeasure(ref a);
            }
            ref var b = ref node->B;
            if (b.Index >= 0)
            {
                childChange += RefitAndMeasure(ref b);
            }
            BoundingBox.CreateMerged(ref a.Min, ref a.Max, ref b.Min, ref b.Max, out child.Min, out child.Max);

            var postmetric = ComputeBoundsMetric(ref child.Min, ref child.Max);
            return postmetric - premetric + childChange; //TODO: would clamping produce a superior result?

        }

        unsafe float RefitAndMark(ref NodeChild child, int leafCountThreshold, ref QuickList<int, Buffer<int>> refinementCandidates, BufferPool<int> intPool)
        {
            Debug.Assert(leafCountThreshold > 1);

            var node = nodes + child.Index;
            Debug.Assert(node->ChildCount >= 2);
            Debug.Assert(node->RefineFlag == 0);
            float childChange = 0;

            var premetric = ComputeBoundsMetric(ref child.Min, ref child.Max);
            //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
            //Add them to a list of refinement candidates.
            //Note that leaves are not included, since they can't be refinement candidates.
            ref var a = ref node->A;
            if (a.Index >= 0)
            {
                if (a.LeafCount <= leafCountThreshold)
                {
                    refinementCandidates.Add(a.Index, intPool);
                    childChange += RefitAndMeasure(ref a);
                }
                else
                {
                    childChange += RefitAndMark(ref a, leafCountThreshold, ref refinementCandidates, intPool);
                }
            }
            ref var b = ref node->B;
            if (b.Index >= 0)
            {
                if (b.LeafCount <= leafCountThreshold)
                {
                    refinementCandidates.Add(b.Index, intPool);
                    childChange += RefitAndMeasure(ref b);
                }
                else
                {
                    childChange += RefitAndMark(ref b, leafCountThreshold, ref refinementCandidates, intPool);
                }
            }

            BoundingBox.CreateMerged(ref a.Min, ref a.Max, ref b.Min, ref b.Max, out child.Min, out child.Max);


            var postmetric = ComputeBoundsMetric(ref child.Min, ref child.Max);

            return postmetric - premetric + childChange; //TODO: Would clamp provide better results?



        }

        unsafe float RefitAndMark(int leafCountThreshold, ref QuickList<int, Buffer<int>> refinementCandidates, BufferPool<int> intPool)
        {
            Debug.Assert(LeafCount > 2, "There's no reason to refit a tree with 2 or less elements. Nothing would happen.");

            var children = &nodes->A;
            float childChange = 0;
            BoundingBox merged = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            for (int i = 0; i < 2; ++i)
            {
                //Note: these conditions mean the root will never be considered a wavefront node. That's acceptable;
                //it will be included regardless.
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    if (child.LeafCount <= leafCountThreshold)
                    {
                        //The wavefront of internal nodes is defined by the transition from more than threshold to less than threshold.
                        //Since we don't traverse into these children, there is no need to check the parent's leaf count.
                        refinementCandidates.Add(child.Index, intPool);
                        childChange += RefitAndMeasure(ref child);
                    }
                    else
                    {
                        childChange += RefitAndMark(ref child, leafCountThreshold, ref refinementCandidates, intPool);
                    }
                }
                BoundingBox.CreateMerged(ref child.Min, ref child.Max, ref merged.Min, ref merged.Max, out merged.Min, out merged.Max);
            }

            var postmetric = ComputeBoundsMetric(ref merged);

            //Note that the root's own change is not included.
            //This cost change is used to determine whether or not to refine.
            //Since refines are unable to change the volume of the root, there's
            //no point in including it in the volume change.
            //It does, however, normalize the child volume changes into a cost metric.
            if (postmetric >= 1e-10)
            {
                return childChange / postmetric;
            }
            return 0;
        }




        unsafe void ValidateRefineFlags(int index)
        {
            var node = nodes + index;
            if (node->RefineFlag != 0)
                Console.WriteLine("Bad refine flag");

            var children = &node->A;
            for (int i = 0; i < node->ChildCount; ++i)
            {
                ref var child = ref children[i];
                if (child.Index >= 0)
                {
                    ValidateRefineFlags(child.Index);
                }
            }
        }

        void GetRefitAndMarkTuning(out int maximumSubtrees, out int estimatedRefinementCandidateCount, out int leafCountThreshold)
        {
            maximumSubtrees = (int)(Math.Sqrt(leafCount) * 3);
            estimatedRefinementCandidateCount = (leafCount * 2) / maximumSubtrees;

            leafCountThreshold = Math.Min(leafCount, maximumSubtrees);
        }


        void GetRefineTuning(int frameIndex, int refinementCandidatesCount, float refineAggressivenessScale, float costChange, int threadCount,
            out int targetRefinementCount, out int refinementPeriod, out int refinementOffset)
        {
            var refineAggressiveness = Math.Max(0, costChange * refineAggressivenessScale);
            float refinePortion = Math.Min(1, refineAggressiveness * 0.25f);



            var targetRefinementScale = Math.Min(nodeCount, Math.Max(2, (float)Math.Ceiling(refinementCandidatesCount * 0.03f)) + refinementCandidatesCount * refinePortion);
            //Round up to the next multiple of the thread count to keep all threads fed.
            //Note that the refinementCandidatesCount is used as a maximum instead of refinementCandidates + 1 for simplicity, since there's a chance
            //that the root would already be a refinementCandidate. Doesn't really have a significant effect either way.
            //var clampInterval = Math.Max(1, threadCount / 2);
            //targetRefinementScale = Math.Min((float)Math.Ceiling(targetRefinementScale / clampInterval) * clampInterval, refinementCandidatesCount);
            refinementPeriod = Math.Max(1, (int)(refinementCandidatesCount / targetRefinementScale));
            refinementOffset = (int)((frameIndex * 236887691L + 104395303L) % Math.Max(1, refinementCandidatesCount));


            targetRefinementCount = (int)targetRefinementScale;
        }

        public int GetCacheOptimizeTuning(int maximumSubtrees, float costChange, float cacheOptimizeAggressivenessScale)
        {
            var cacheOptimizeAggressiveness = Math.Max(0, costChange * cacheOptimizeAggressivenessScale);
            float cacheOptimizePortion = Math.Min(1, 0.03f + 85f * (maximumSubtrees / (float)leafCount) * cacheOptimizeAggressiveness);
            //float cacheOptimizePortion = Math.Min(1, 0.03f + cacheOptimizeAggressiveness * 0.5f);
            //Console.WriteLine($"cache optimization portion: {cacheOptimizePortion}");
            return (int)Math.Ceiling(cacheOptimizePortion * nodeCount);
        }



        public unsafe int RefitAndRefine(int frameIndex, float refineAggressivenessScale = 1, float cacheOptimizeAggressivenessScale = 1)
        {
            //Don't proceed if the tree is empty.
            if (leafCount == 0)
                return 0;
            int maximumSubtrees, estimatedRefinementCandidateCount, leafCountThreshold;
            GetRefitAndMarkTuning(out maximumSubtrees, out estimatedRefinementCandidateCount, out leafCountThreshold);

            var refinementCandidates = new QuickList<int>(pool, BufferPool<int>.GetPoolIndex(estimatedRefinementCandidateCount));

            //Collect the refinement candidates.
            var costChange = RefitAndMark(leafCountThreshold, ref refinementCandidates);


            int targetRefinementCount, period, offset;
            GetRefineTuning(frameIndex, refinementCandidates.Count, refineAggressivenessScale, costChange, 1, out targetRefinementCount, out period, out offset);


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
                nodes[refinementCandidates.Elements[index]].RefineFlag = 1;
            }
            refinementTargets.Count = actualRefinementTargetsCount;
            refinementCandidates.Count = 0;
            refinementCandidates.Dispose();
            if (nodes->RefineFlag == 0)
            {
                refinementTargets.Add(0);
                nodes->RefineFlag = 1;
                ++actualRefinementTargetsCount;
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

            var cacheOptimizeCount = GetCacheOptimizeTuning(maximumSubtrees, costChange, cacheOptimizeAggressivenessScale);

            var startIndex = (int)(((long)frameIndex * cacheOptimizeCount) % nodeCount);

            //We could wrap around. But we could also not do that because it doesn't really matter!
            //var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            var end = Math.Min(NodeCount, startIndex + cacheOptimizeCount);
            for (int i = startIndex; i < end; ++i)
            {
                IncrementalCacheOptimize(i);
            }
            //var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
            //Console.WriteLine($"Cache optimize time: {endTime - startTime}");

            return actualRefinementTargetsCount;
        }




    }
}
