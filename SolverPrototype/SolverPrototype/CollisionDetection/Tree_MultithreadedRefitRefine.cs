using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Threading;

namespace SolverPrototype.CollisionDetection
{
    partial class Tree
    {
        /// <summary>
        /// Caches input and output for the multithreaded execution of a tree's refit and refinement operations.
        /// </summary>
        public class RefitAndRefineMultithreadedContext
        {
            Tree Tree;
            
            int RefitNodeIndex;
            QuickList<int, Buffer<int>> RefitNodes;
            float RefitCostChange;

            int LeafCountThreshold;
            Buffer<QuickList<int, Buffer<int>>> RefinementCandidates;
            Action<int> RefitAndMarkAction;

            int RefineIndex;
            QuickList<int, Buffer<int>> RefinementTargets;
            int MaximumSubtrees;
            Action<int> RefineAction;

            QuickList<int, Buffer<int>> CacheOptimizeStarts;
            int PerWorkerCacheOptimizeCount;
            Action<int> CacheOptimizeAction;

            IThreadDispatcher threadDispatcher;

            public RefitAndRefineMultithreadedContext()
            {
                RefitAndMarkAction = RefitAndMark;
                RefineAction = Refine;
                CacheOptimizeAction = CacheOptimize;
            }

            public void RefitAndRefine(Tree tree, IThreadDispatcher threadDispatcher)
            {
                if (tree.leafCount <= 2)
                {
                    //If there are 2 or less leaves, then refit/refine/cache optimize doesn't do anything at all.
                    //(The root node has no parent, so it does not have a bounding box, and the SAH won't change no matter how we swap the children of the root.)
                    //Avoiding this case also gives the other codepath a guarantee that it will be working with nodes with two children.
                    return;
                }
                this.threadDispatcher = threadDispatcher;
                Tree = tree;
                //Note that we create per-thread refinement candidates. That's because candidates are found during the multithreaded refit and mark phase, and 
                //we don't want to spend the time doing sync work. The candidates are then pruned down to a target single target set for the refine pass.
                Tree.Pool.SpecializeFor<QuickList<int, Buffer<int>>>().Take(threadDispatcher.ThreadCount, out RefinementCandidates);
                tree.GetRefitAndMarkTuning(out MaximumSubtrees, out var LeafCountThreshold, out var refinementLeafCountThreshold);
                //Note that we haven't rigorously guaranteed a refinement count maximum, so it's possible that the other threads will need to resize the per-thread refinement candidate lists.
                //So we don't create the refinement candidate thread lists here; we let the worker do it.


                int multithreadingLeafCountThreshold = Tree.leafCount / (threadDispatcher.ThreadCount * 2);
                if (multithreadingLeafCountThreshold < refinementLeafCountThreshold)
                    multithreadingLeafCountThreshold = refinementLeafCountThreshold;
                CollectNodesForMultithreadedRefit(0, multithreadingLeafCountThreshold, ref RefitNodes, refinementLeafCountThreshold, ref RefinementCandidates[0], 
                    threadDispatcher.GetThreadMemoryPool(0).SpecializeFor<int>());

                threadDispatcher.DispatchWorkers(RefitAndMarkAction);



                RefitNodeIndex = -1;
                RefineIndex = -1;
                for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                {
                    //Note the use of the thread memory pool. Each thread allocated their own memory for the list since resizes were possible.
                    RefinementCandidates[i].Dispose(threadDispatcher.GetThreadMemoryPool(i).SpecializeFor<int>());
                }
                Tree.Pool.SpecializeFor<QuickList<int, Buffer<int>>>().Return(ref RefinementCandidates);
                Tree = null;
                this.threadDispatcher = null;
            }

            unsafe void CollectNodesForMultithreadedRefit(int nodeIndex,
                int multithreadingLeafCountThreshold, ref QuickList<int, Buffer<int>> refitAndMarkTargets,
                int refinementLeafCountThreshold, ref QuickList<int, Buffer<int>> refinementCandidates, BufferPool<int> intPool)
            {
                var node = Tree.nodes + nodeIndex;
                var children = &node->A;
                Debug.Assert(node->RefineFlag == 0);
                for (int i = 0; i < node->ChildCount; ++i)
                {
                    ref var child = ref children[i];
                    if (child.Index >= 0)
                    {
                        //Each node stores how many children are involved in the multithreaded refit.
                        //This allows the postphase to climb the tree in a thread safe way.
                        ++node->RefineFlag;
                        if (child.LeafCount <= multithreadingLeafCountThreshold)
                        {
                            if (child.LeafCount <= refinementLeafCountThreshold)
                            {
                                //It's possible that a wavefront node is this high in the tree, so it has to be captured here because the postpass won't find it.
                                refinementCandidates.Add(child.Index, intPool);
                                //Encoding the child index tells the thread to use RefitAndMeasure instead of RefitAndMark since this was a wavefront node.
                                refitAndMarkTargets.Add(Encode(child.Index), intPool);
                            }
                            else
                            {
                                refitAndMarkTargets.Add(child.Index, intPool);
                            }
                        }
                        else
                        {
                            CollectNodesForMultithreadedRefit(child.Index, multithreadingLeafCountThreshold, ref refitAndMarkTargets, refinementLeafCountThreshold, ref refinementCandidates, intPool);
                        }
                    }
                }
            }

            unsafe void RefitAndMark(int workerIndex)
            {
                //Since resizes may occur, we have to use the thread's buffer pool.
                var threadIntPool = threadDispatcher.GetThreadMemoryPool(workerIndex).SpecializeFor<int>();
                QuickList<int, Buffer<int>>.Create(threadIntPool, LeafCountThreshold, out RefinementCandidates[workerIndex]);
                int refitIndex;
                while ((refitIndex = Interlocked.Increment(ref RefitNodeIndex)) < RefitNodes.Count)
                {

                    var nodeIndex = RefitNodes[refitIndex];
                    bool shouldUseMark;
                    if (nodeIndex < 0)
                    {
                        //Node was already marked as a wavefront. Should proceed with a RefitAndMeasure instead of RefitAndMark.
                        nodeIndex = Encode(nodeIndex);
                        shouldUseMark = false;
                    }
                    else
                    {
                        shouldUseMark = true;
                    }

                    var node = Tree.nodes + nodeIndex;
                    Debug.Assert(node->Parent >= 0, "The root should not be marked for refit.");
                    var parent = Tree.nodes + node->Parent;
                    var childInParent = &parent->A + node->IndexInParent;
                    if (shouldUseMark)
                    {
                        var costChange = Tree.RefitAndMark(ref *childInParent, LeafCountThreshold, ref RefinementCandidates[workerIndex], threadIntPool);
                        node->LocalCostChange = costChange;
                    }
                    else
                    {
                        var costChange = Tree.RefitAndMeasure(ref *childInParent);
                        node->LocalCostChange = costChange;
                    }


                    //int foundLeafCount;
                    //Tree.Validate(RefitNodes.Elements[refitNodeIndex], node->Parent, node->IndexInParent, ref *boundingBoxInParent, out foundLeafCount);


                    //Walk up the tree.
                    node = parent;
                    while (true)
                    {

                        if (Interlocked.Decrement(ref node->RefineFlag) == 0)
                        {
                            //Compute the child contributions to this node's volume change.
                            var children = &node->A;
                            node->LocalCostChange = 0;
                            for (int i = 0; i < node->ChildCount; ++i)
                            {
                                ref var child = ref children[i];
                                if (child.Index >= 0)
                                {
                                    var childMetadata = Tree.nodes + child.Index;
                                    node->LocalCostChange += childMetadata->LocalCostChange;
                                    //Clear the refine flag (unioned).
                                    childMetadata->RefineFlag = 0;

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
                                for (int i = 0; i < 2; ++i)
                                {
                                    ref var child = ref children[i];
                                    BoundingBox.CreateMerged(ref child.Min, ref child.Max, ref merged.Min, ref merged.Max, out merged.Min, out merged.Max);
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
                                childInParent = &parent->A + node->IndexInParent;
                                var premetric = ComputeBoundsMetric(ref childInParent->Min, ref childInParent->Max);
                                childInParent->Min = new Vector3(float.MaxValue);
                                childInParent->Max = new Vector3(float.MinValue);
                                for (int i = 0; i < 2; ++i)
                                {
                                    ref var child = ref children[i];
                                    BoundingBox.CreateMerged(ref child.Min, ref child.Max, ref childInParent->Min, ref childInParent->Max, out childInParent->Min, out childInParent->Max);
                                }
                                var postmetric = ComputeBoundsMetric(ref childInParent->Min, ref childInParent->Max);
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
                var threadPool = threadDispatcher.GetThreadMemoryPool(workerIndex);
                var threadIntPool = threadPool.SpecializeFor<int>();
                var subtreeCountEstimate = 1 << SpanHelper.GetContainingPowerOf2(MaximumSubtrees);
                QuickList<int, Buffer<int>>.Create(threadIntPool, subtreeCountEstimate, out var subtreeReferences);
                QuickList<int, Buffer<int>>.Create(threadIntPool, subtreeCountEstimate, out var treeletInternalNodes);
                
                int refineIndex;
                while ((refineIndex = Interlocked.Increment(ref RefineIndex)) < RefinementTargets.Count)
                {
                    subtreeReferences.Count = 0;
                    treeletInternalNodes.Count = 0;
                    Tree.BinnedRefine(RefinementTargets[refineIndex], ref subtreeReferences, MaximumSubtrees, ref treeletInternalNodes, ref threadPool);
                    //Allow other refines to traverse this node.
                    Tree.nodes[RefinementTargets[refineIndex]].RefineFlag = 0;
                }
                
                subtreeReferences.Dispose(threadIntPool);
                treeletInternalNodes.Dispose(threadIntPool);


            }

            void CacheOptimize(int workerIndex)
            {
                var startIndex = CacheOptimizeStarts[workerIndex];

                //We could wrap around. But we could also not do that because it doesn't really matter!
                var end = Math.Min(Tree.nodeCount, startIndex + PerWorkerCacheOptimizeCount);
                for (int i = startIndex; i < end; ++i)
                {
                    Tree.IncrementalCacheOptimizeThreadSafe(i);
                }

            }
        }



        unsafe void CheckForRefinementOverlaps(int nodeIndex, ref QuickList<int, Buffer<int>> refinementTargets)
        {

            var node = nodes + nodeIndex;
            var children = &node->A;
            for (int childIndex = 0; childIndex < node->ChildCount; ++childIndex)
            {
                ref var child = ref children[childIndex];
                if (child.Index >= 0)
                {
                    for (int i = 0; i < refinementTargets.Count; ++i)
                    {
                        if (refinementTargets[i] == child.Index)
                            Console.WriteLine("Found a refinement target in the children of a refinement target.");
                    }

                    CheckForRefinementOverlaps(child.Index, ref refinementTargets);
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

            //Collect the refinement candidates.
            if (LeafCount <= 2)
            {
                RefitAndMark(context.LeafCountThreshold, ref context.RefinementCandidates.Elements[0]);
            }
            else
            {
                CollectNodesForMultithreadedRefit(looper.ThreadCount, ref context.RefitNodes, context.LeafCountThreshold, ref context.RefinementCandidates.Elements[0]);
                //Console.WriteLine($"Refit subtree count: {context.RefitNodes.Count}");
                looper.ForLoop(0, looper.ThreadCount, context.RefitAndMarkAction);
            }

            var refinementCandidatesCount = 0;
            for (int i = 0; i < looper.ThreadCount; ++i)
            {
                refinementCandidatesCount += context.RefinementCandidates.Elements[i].Count;
            }

            int targetRefinementCount, period, offset;
            GetRefineTuning(frameIndex, refinementCandidatesCount, refineAggressivenessScale, context.RefitCostChange, looper.ThreadCount, out targetRefinementCount, out period, out offset);



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



            //Refine all marked targets.
            looper.ForLoop(0, Math.Min(looper.ThreadCount, context.RefinementTargets.Count), context.RefineAction);


            //There's no reason to perform any kind of cache optimization on a tree with less than 2 leaf nodes.
            //That, combined with using a binary tree, gives us a guarantee: every node has 2 children. No need for per-node child counts.
            if (leafCount > 2)
            {
                //To multithread this, give each worker a contiguous chunk of nodes. You want to do the biggest chunks possible to chain decent cache behavior as far as possible.
                //Note that more cache optimization is required with more threads, since spreading it out more slightly lessens its effectiveness.
                var cacheOptimizeCount = GetCacheOptimizeTuning(context.MaximumSubtrees, context.RefitCostChange, (Math.Max(1, looper.ThreadCount * 0.25f)) * cacheOptimizeAggressivenessScale);

                var cacheOptimizationTasks = looper.ThreadCount * 2;
                context.PerWorkerCacheOptimizeCount = cacheOptimizeCount / cacheOptimizationTasks;
                var startIndex = (int)(((long)frameIndex * context.PerWorkerCacheOptimizeCount) % nodeCount);
                context.CacheOptimizeStarts.Add(startIndex);

                var optimizationSpacing = nodeCount / looper.ThreadCount;
                var optimizationSpacingWithExtra = optimizationSpacing + 1;
                var optimizationRemainder = nodeCount - optimizationSpacing * looper.ThreadCount;

                for (int i = 1; i < cacheOptimizationTasks; ++i)
                {
                    if (optimizationRemainder > 0)
                    {
                        startIndex += optimizationSpacingWithExtra;
                        --optimizationRemainder;
                    }
                    else
                    {
                        startIndex += optimizationSpacing;
                    }
                    if (startIndex >= nodeCount)
                        startIndex -= nodeCount;
                    Debug.Assert(startIndex >= 0 && startIndex < nodeCount);
                    context.CacheOptimizeStarts.Add(startIndex);
                }

                //for (int i = 0; i < looper.ThreadCount; ++i)
                //{
                //    var start = context.CacheOptimizeStarts[i];
                //    var end = Math.Min(start + context.PerWorkerCacheOptimizeCount, NodeCount);
                //    for (int j = start; j < end; ++j)
                //    {
                //        //ValidateRefineFlags(0);
                //        IncrementalCacheOptimizeThreadSafe(j);
                //        //ValidateRefineFlags(0);
                //    }
                //}


                //var start = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //Validate();
                //ValidateRefineFlags(0);
                looper.ForLoop(0, cacheOptimizationTasks, context.CacheOptimizeAction);
                //ValidateRefineFlags(0);
                //var end = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                //Validate();
                //Console.WriteLine($"Cache optimize time: {end - start}");
            }


            context.CleanUp();
            return actualRefinementTargetsCount;
        }


    }
}
