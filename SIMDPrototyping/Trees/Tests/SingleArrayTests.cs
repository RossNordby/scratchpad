using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


using SIMDPrototyping.Trees.SingleArray;
using System.Numerics;
using System.Diagnostics;

namespace SIMDPrototyping.Trees.Tests
{
    partial class TreeTest
    {
        public unsafe static void TestSingleArray(TestCollidable[] leaves, BoundingBox[] queries, BoundingBox positionBounds, int queryCount, int selfTestCount, int refitCount)
        {
            {

                var warmLeaves = GetLeaves(10, 10, 10, 10, 10);
                Tree tree = new Tree();
                //for (int i = 0; i < leaves.Length; ++i)
                //{
                //    BoundingBox box;
                //    leaves[i].GetBoundingBox(out box);
                //    //tree.Insert(i, ref box);
                //    tree.AddGlobal(i, ref box);
                //}
                int[] leafIds = new int[warmLeaves.Length];
                BoundingBox[] leafBounds = new BoundingBox[warmLeaves.Length];
                for (int i = 0; i < warmLeaves.Length; ++i)
                {
                    leafIds[i] = i;
                    warmLeaves[i].GetBoundingBox(out leafBounds[i]);
                }
                //tree.BuildMedianSplit(leafIds, leafBounds);
                //tree.BuildVolumeHeuristic(leafIds, leafBounds);
                tree.SweepBuild(leafIds, leafBounds);
                Console.WriteLine($"SingleArray Cachewarm Build: {tree.LeafCount}");

                tree.Refit();
                //tree.BottomUpAgglomerativeRefine();
                //tree.TopDownAgglomerativeRefine();
                tree.BottomUpSweepRefine();
                tree.TopDownSweepRefine();

                var list = new QuickList<int>(new BufferPool<int>());
                BoundingBox aabb = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1, 1, 1) };
                tree.QueryRecursive(ref aabb, ref list);
                list.Dispose();

                var overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                tree.GetSelfOverlaps(ref overlaps);

                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                tree.GetSelfOverlapsArityDedicated(ref overlaps);

                tree.IncrementalCacheOptimize(0);

                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());

                tree.GetSelfOverlapsViaQueries(ref overlaps);
                Console.WriteLine($"Cachewarm overlaps: {overlaps.Count}");
                tree.Dispose();

            }

            {
                Console.WriteLine($"SingleArray arity: {Tree.ChildrenCapacity}");
                Tree tree = new Tree(leaves.Length);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    var leafIndex = (int)((982451653L * i) % leaves.Length);
                    BoundingBox box;
                    leaves[leafIndex].GetBoundingBox(out box);
                    tree.Add(leafIndex, ref box);
                    //tree.AddGlobal(leafIndex, ref box);
                }
                //int[] leafIds = new int[leaves.Length];
                //BoundingBox[] leafBounds = new BoundingBox[leaves.Length];
                //for (int i = 0; i < leaves.Length; ++i)
                //{
                //    leafIds[i] = i;
                //    leaves[i].GetBoundingBox(out leafBounds[i]);
                //}
                ////tree.BuildMedianSplit(leafIds, leafBounds);
                ////tree.BuildVolumeHeuristic(leafIds, leafBounds);
                //tree.SweepBuild(leafIds, leafBounds);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Build Time: {endTime - startTime}, depth: {tree.ComputeMaximumDepth()}");

                int nodeCount, childCount;
                tree.MeasureNodeOccupancy(out nodeCount, out childCount);
                Console.WriteLine($"SingleArray Occupancy: {childCount / (double)nodeCount}");
                Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");

                tree.Validate();

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < refitCount; ++i)
                {
                    //for (int i = 0; i < tree.LeafCount; ++i)
                    //{
                    //    BoundingBox box;
                    //    leaves[tree.Leaves[i].Id].GetBoundingBox(out box);
                    //    tree.UpdateLeafBoundingBox(i, ref box);
                    //}
                    tree.Refit();
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Refit Time1: {endTime - startTime}");

                var overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlaps(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray SelfTree Time1: {endTime - startTime}, overlaps: {overlaps.Count}");

                Console.WriteLine($"SingleArray Cache Quality Before: {tree.MeasureCacheQuality()}");

                int[] buffer;
                MemoryRegion region;
                BinnedResources resources;
                const int maximumSubtrees = 262144;
                var spareNodes = new QuickList<int>(new BufferPool<int>(), 8);
                var subtreeReferences = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
                var treeletInternalNodes = new QuickQueue<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
                var refinementTargets = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex((int)(tree.LeafCount / (maximumSubtrees * 0.5f))));
                RawList<int> treeletInternalNodesCopy = new RawList<int>(maximumSubtrees);
                Tree.CreateBinnedResources(BufferPools<int>.Thread, maximumSubtrees, out buffer, out region, out resources);
                bool nodesInvalidated;
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                //for (int i = 0; i < 5; ++i)
                //{
                //    spareNodes.Count = 0;


                //    //tree.SweepRefine(0, ref spareNodes, out nodesInvalidated);
                //    //tree.BinnedRefine(0, ref spareNodes, maximumSubtrees, ref resources, out nodesInvalidated);

                //    tree.BottomUpBinnedRefine(maximumSubtrees);
                //    tree.TopDownBinnedRefine(maximumSubtrees);
                //    //tree.BottomUpSweepRefine();
                //    //tree.TopDownSweepRefine();
                //    //tree.BottomUpAgglomerativeRefine();
                //    //tree.Refit();
                //    //tree.BottomUpRefine();
                //    //Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");
                //    //tree.Validate();
                //}


                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Refine Time: {endTime - startTime}");

                Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");
                Console.WriteLine($"SingleArray Cache Quality: {tree.MeasureCacheQuality()}");

                var visitedNodes = new QuickSet<int>(BufferPools<int>.Thread, BufferPools<int>.Thread);

                //**************** Incremental Testing
                //for (int i = 0; i < leaves.Length - 1; ++i)
                //{
                //    tree.IncrementalCacheOptimizeMultithreaded(i);
                //}
                //tree.Validate();
                //var oldTree = tree;
                //tree = tree.CreateOptimized();
                //oldTree.Dispose();
                //tree.Validate();
                //Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");
                Random random = new Random(5);
                const float maxVelocity = 0;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    leaves[i].Velocity = maxVelocity * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * 2 - Vector3.One);
                }
                const float dt = 1f / 60f;
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int t = 0; t < 4096; ++t)
                {
                    //Update the positions of objects.
                    for (int i = 0; i < tree.LeafCount; ++i)
                    {
                        var leafId = tree.Leaves[i].Id;
                        var leaf = leaves[leafId];

                        //Bounce off the walls.
                        if (leaf.Position.X < positionBounds.Min.X && leaf.Velocity.X < 0)
                            leaf.Velocity.X = -leaf.Velocity.X;
                        if (leaf.Position.Y < positionBounds.Min.Y && leaf.Velocity.Y < 0)
                            leaf.Velocity.Y = -leaf.Velocity.Y;
                        if (leaf.Position.Z < positionBounds.Min.Z && leaf.Velocity.Z < 0)
                            leaf.Velocity.Z = -leaf.Velocity.Z;

                        if (leaf.Position.X > positionBounds.Max.X && leaf.Velocity.X > 0)
                            leaf.Velocity.X = -leaf.Velocity.X;
                        if (leaf.Position.Y > positionBounds.Max.Y && leaf.Velocity.Y > 0)
                            leaf.Velocity.Y = -leaf.Velocity.Y;
                        if (leaf.Position.Z > positionBounds.Max.Z && leaf.Velocity.Z > 0)
                            leaf.Velocity.Z = -leaf.Velocity.Z;

                        leaf.Position += leaf.Velocity * dt;
                        BoundingBox boundingBox;
                        leaf.GetBoundingBox(out boundingBox);
                        tree.SetLeafBoundingBox(i, ref boundingBox);
                    }
                    var startTimeInner = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                    //List<int> refinementTargets2 = new List<int>();
                    //{
                    //    const int skip = 8;
                    //    var startIndex = (int)((t * 79151L) % skip);
                    //    for (int i = startIndex; i < tree.NodeCount; i += skip)
                    //    {
                    //        var node = tree.Nodes[i];
                    //        var leafCounts = &node.LeafCountA;
                    //        int leafCount = 0;
                    //        for (int childIndex = 0; childIndex < node.ChildCount; ++childIndex)
                    //        {
                    //            leafCount += leafCounts[childIndex];
                    //        }

                    //        if (leafCount >= Math.Min(tree.LeafCount, maximumSubtrees * 0.75f))
                    //        {
                    //            refinementTargets2.Add(i);
                    //        }
                    //    }
                    //}


                    //refinementTargets.Count = 0;
                    //tree.RefitAndRefine(ref refinementTargets, t, 1, maximumSubtrees);
                    //Console.WriteLine($"Tree depth: {tree.ComputeMaximumDepth()}");
                    //Console.WriteLine($"Refinement count: {refinementTargets.Count}");



                    //Array.Sort(refinementTargets.Elements, 0, refinementTargets.Count);
                    //refinementTargets2.Sort();

                    //for (int j = 0; j < refinementTargets.Count; ++j)
                    //{
                    //    if (refinementTargets[j] != refinementTargets2[j])
                    //        Console.WriteLine("bad");
                    //}


                    if (false)
                    {
                        tree.Refit();


                        const int skip = 8;
                        var startIndex = (int)((t * 79151L) % skip);
                        int numberOfDuplicates = 0;
                        int refinementCount = 0;
                        for (int i = startIndex; i < tree.NodeCount; i += skip)
                        {
                            subtreeReferences.Count = 0;
                            treeletInternalNodes.FastClear();
                            //Avoid refitting any node which doesn't have enough children to warrant a full refine.
                            var node = tree.Nodes[i];
                            var leafCounts = &node.LeafCountA;
                            int leafCount = 0;
                            for (int childIndex = 0; childIndex < node.ChildCount; ++childIndex)
                            {
                                leafCount += leafCounts[childIndex];
                            }

                            if (leafCount >= Math.Min(tree.LeafCount, maximumSubtrees * 0.75f))
                            {
                                //tree.BinnedRefine(i, ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);
                                tree.BinnedRefine(i, ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated, treeletInternalNodesCopy);
                                ++refinementCount;
                                for (int internalNodeIndex = 0; internalNodeIndex < treeletInternalNodesCopy.Count; ++internalNodeIndex)
                                {
                                    if (!visitedNodes.Add(treeletInternalNodesCopy[internalNodeIndex]))
                                    {
                                        ++numberOfDuplicates;
                                    }
                                }
                            }
                        }
                        Console.WriteLine($"Tree depth: {tree.ComputeMaximumDepth()}");
                        Console.WriteLine($"Refinement count: {refinementCount}");
                        Console.WriteLine($"Fraction of internal nodes visited: {visitedNodes.Count / (double)tree.NodeCount}");
                        Console.WriteLine($"Fraction of duplicates visited: {(visitedNodes.Count > 0 ? (numberOfDuplicates / (double)visitedNodes.Count) : 0)}");
                        visitedNodes.FastClear();


                        //tree.PartialRefine(t, 2000, ref spareNodes, maximumSubtrees, ref resources, out nodesInvalidated);


                        //tree.RecursiveRefine(maximumSubtrees, t, ref spareNodes, ref resources, out nodesInvalidated);


                        //subtreeReferences.Count = 0;
                        //treeletInternalNodes.FastClear();
                        //tree.BinnedRefine(0, ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);


                        tree.RemoveUnusedInternalNodes(ref spareNodes);


                        //const int skip = 4;
                        //var startIndex = (int)((t * 79151L) % skip);
                        //for (int i = startIndex; i < tree.NodeCount; i += skip)
                        //{
                        //    subtreeReferences.Count = 0;
                        //    var leaf = tree.Leaves[i];
                        //    BoundingBox boundingBox;
                        //    tree.GetLeafBoundingBox(i, out boundingBox);
                        //    tree.RemoveAt(i);

                        //    tree.Add(leaf.Id, ref boundingBox);


                        //}
                    }
                    tree.Refit();
                    startTimeInner = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    for (int i = 0; i < 10; ++i)
                    {

                        subtreeReferences.Count = 0;
                        treeletInternalNodes.FastClear();
                        tree.BinnedRefine(0, ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);
                    }
                    tree.RemoveUnusedInternalNodes(ref spareNodes);
                    var endTimeInner = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                    //startTimeInner = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                    {

                        const int skip = 1024;
                        const int intervalLength = 1024;
                        var startIndex = (t * intervalLength) % skip;
                        for (int i = startIndex; i < tree.NodeCount; i += skip)
                        {
                            var end = Math.Min(tree.NodeCount, i + intervalLength);
                            for (int j = i; j < end; ++j)
                            {
                                tree.IncrementalCacheOptimizeMultithreaded(j);
                                //tree.IncrementalCacheOptimize(j);
                            }
                        }
                    }
                    if (t % 16 == 0)
                    {
                        Console.WriteLine($"Cache Quality {t}: {tree.MeasureCacheQuality()}");
                        Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");
                        Console.WriteLine($"Refine/Optimize Time: {endTimeInner - startTimeInner}");

                    }
                    tree.Validate();
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Validate();
                Console.WriteLine($"Incremental Cache Optimize Time: {endTime - startTime}");
                Console.WriteLine($"SingleArray Cache Quality: {tree.MeasureCacheQuality()}");
                Console.WriteLine($"Cost metric: {tree.MeasureCostMetric()}");

                region.Dispose();
                tree.RemoveUnusedInternalNodes(ref spareNodes);
                BufferPools<int>.Thread.GiveBack(buffer);

                //********************

                tree.MeasureNodeOccupancy(out nodeCount, out childCount);
                Console.WriteLine($"SingleArray Occupancy: {childCount / (double)nodeCount}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < refitCount; ++i)
                {
                    //for (int i = 0; i < tree.LeafCount; ++i)
                    //{
                    //    BoundingBox box;
                    //    leaves[tree.Leaves[i].Id].GetBoundingBox(out box);
                    //    tree.UpdateLeafBoundingBox(i, ref box);
                    //}
                    tree.Refit();
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Refit Time2: {endTime - startTime}");


                var list = new QuickList<int>(new BufferPool<int>());
                var queryMask = queries.Length - 1;
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < queryCount; ++i)
                {
                    list.Count = 0;
                    //tree.Query2(ref queries[i & queryMask], ref list);
                    tree.QueryRecursive(ref queries[i & queryMask], ref list);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Query Time: {endTime - startTime}, overlaps: {list.Count}");
                Array.Clear(list.Elements, 0, list.Elements.Length);
                list.Dispose();

                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlaps(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray SelfTree Time: {endTime - startTime}, overlaps: {overlaps.Count}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlapsArityDedicated(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Arity-Dedicated SelfTree Time: {endTime - startTime}, overlaps: {overlaps.Count}");


                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlapsViaQueries(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray SelfQuery Time: {endTime - startTime}, overlaps: {overlaps.Count}");

                tree.Dispose();
            }
        }
    }
}
