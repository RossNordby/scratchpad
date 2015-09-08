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
        
        public unsafe static TestResults TestSingleArray(TestCollidable[] leaves, BoundingBox[] queries, BoundingBox positionBounds,
            int queryCount, int selfTestCount, int refitCount, int frameCount, float dt)
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
                tree.RefitAndRefine(0);

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
                Console.WriteLine($"Cache Quality: {tree.MeasureCacheQuality()}");

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
                

                int[] buffer;
                MemoryRegion region;
                BinnedResources resources;
                const int maximumSubtrees = 262144;
                var spareNodes = new QuickList<int>(new BufferPool<int>(), 8);
                var subtreeReferences = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
                var treeletInternalNodes = new QuickList<int>(BufferPools<int>.Thread, BufferPool<int>.GetPoolIndex(maximumSubtrees));
                Tree.CreateBinnedResources(BufferPools<int>.Thread, maximumSubtrees, out buffer, out region, out resources);
                bool nodesInvalidated;
                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                

                var visitedNodes = new QuickSet<int>(BufferPools<int>.Thread, BufferPools<int>.Thread);

                //**************** Dynamic Testing
                Random random = new Random(5);
                TestResults results = new TestResults("NewTree Dynamic", frameCount);
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int t = 0; t < frameCount; ++t)
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
                    var refineStartTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;


                    var refinementCount = tree.RefitAndRefine(t);


                    //tree.Refit();
                    //for (int i = 0; i < 1; ++i)
                    //{

                    //    subtreeReferences.Count = 0;
                    //    treeletInternalNodes.Count = 0;
                    //    tree.BinnedRefine(0, ref subtreeReferences, maximumSubtrees, ref treeletInternalNodes, ref spareNodes, ref resources, out nodesInvalidated);
                    //}
                    //tree.RemoveUnusedInternalNodes(ref spareNodes);

                    var refineEndTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                    overlaps.Count = 0;
                    tree.GetSelfOverlapsArityDedicated(ref overlaps);

                    var testEndTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;

                    results.Refine[t] = 1000 * (refineEndTime - refineStartTime);
                    results.SelfTest[t] = 1000 * (testEndTime - refineEndTime);
                    results.Total[t] = 1000 * (testEndTime - refineStartTime);
                    results.OverlapCounts[t] = overlaps.Count;
                    results.TreeCosts[t] = tree.MeasureCostMetric();

                    if (t % 16 == 0)
                    {
                        Console.WriteLine($"_________________{t}_________________");
                        Console.WriteLine($"Refinement count: {refinementCount}");
                        Console.WriteLine($"Refine time:      {results.Refine[t]}");
                        Console.WriteLine($"Test time:        {results.SelfTest[t]}");
                        Console.WriteLine($"TIME:             {results.Total[t]}");
                        Console.WriteLine($"Cost metric:      {results.TreeCosts[t]}");
                        Console.WriteLine($"Overlaps:         {results.OverlapCounts[t]}");
                        Console.WriteLine($"Cache Quality:    {tree.MeasureCacheQuality()}");
                        GC.Collect();
                    }
                    tree.Validate();
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Validate();
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

                return results;
            }

        }
    }
}
