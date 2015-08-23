﻿using BEPUutilities.DataStructures;
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
                QuickList<int> spareNodes = new QuickList<int>(new BufferPool<int>(), 8);
                QuickList<int> subtreeReferences = new QuickList<int>(new BufferPool<int>(), BufferPool<int>.GetPoolIndex(maximumSubtrees));
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
                const float maxVelocity = 100;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    leaves[i].Velocity = maxVelocity * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * 2 - Vector3.One);
                }
                const float dt = 1f / 60f;
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int t = 0; t < 1024; ++t)
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

                    tree.Refit();
                    //if (t < 500)
                    {
                        //const int skip = 1024;
                        //var startIndex = (t * 17) % skip;
                        //for (int i = startIndex; i < tree.NodeCount; i += skip)
                        //{
                        //    subtreeReferences.Count = 0;
                        //    tree.BinnedRefine(i, ref subtreeReferences, maximumSubtrees, ref spareNodes, ref resources, out nodesInvalidated);
                        //}

                        subtreeReferences.Count = 0;
                        tree.BinnedRefine(0, ref subtreeReferences, maximumSubtrees, ref spareNodes, ref resources, out nodesInvalidated);

                        //tree.PartialRefine(t, 2000, ref spareNodes, maximumSubtrees, ref resources, out nodesInvalidated);


                        //tree.RecursiveRefine(maximumSubtrees, ref spareNodes, ref resources, out nodesInvalidated);

                        //tree.RefitRefine(maximumSubtrees, 1.1f);
                        tree.RemoveUnusedInternalNodes(ref spareNodes);
                    }

                    {

                        const int skip = 4096;
                        const int intervalLength = 1024;
                        var startIndex = (t * intervalLength) % skip;
                        for (int i = startIndex; i < tree.NodeCount; i += skip)
                        {
                            var end = Math.Min(tree.NodeCount, i + intervalLength);
                            for (int j = i; j < end; ++j)
                            {
                                tree.IncrementalCacheOptimizeMultithreaded(j);
                            }
                        }
                    }
                    var endTimeInner = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
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
