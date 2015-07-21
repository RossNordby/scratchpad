using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


using BaselineTree = SIMDPrototyping.Trees.Baseline.Tree<SIMDPrototyping.Trees.TestCollidable>;
using SIMDPrototyping.Trees.Baseline;
using System.Numerics;
using System.Diagnostics;

namespace SIMDPrototyping.Trees.Tests
{
    partial class TreeTest
    {
        public static void TestBaseline(TestCollidable[] leaves, BoundingBox[] queries, int queryCount, int selfTestCount)
        {
            {
                var warmLeaves = GetLeaves(10, 10, 10, 10, 10);
                BaselineTree tree = new BaselineTree();
                //for (int i = 0; i < leaves.Length; ++i)
                //{
                //    tree.Insert(warmLeaves[i]);
                //}
                tree.BuildMedianSplit(warmLeaves);
                //tree.BuildVolumeHeuristic(warmLeaves);
                Console.WriteLine($"Baseline Cachewarm Build: {tree.LeafCount}");

                tree.RefitLeaves();
                tree.Refit();

                var list = new QuickList<int>(new BufferPool<int>());
                BoundingBox aabb = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1, 1, 1) };
                tree.QueryRecursive(ref aabb, ref list);
                list.Dispose();

                var overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());
                tree.GetSelfOverlaps(ref overlaps);
                Console.WriteLine($"Cachewarm overlaps: {overlaps.Count}");

                overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());

                tree.GetSelfOverlapsViaQueries(ref overlaps);
                Console.WriteLine($"Cachewarm overlaps: {overlaps.Count}");
            }

            {
                Console.WriteLine($"Baseline arity: {BaselineTree.ChildrenCapacity}");
                BaselineTree tree = new BaselineTree(leaves.Length, 32);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //for (int i = 0; i < leaves.Length; ++i)
                //{
                //    tree.Insert(leaves[(int)((982451653L * i) % leaves.Length)]);
                //    //tree.InsertGlobal(leaves[(int)((982451653L * i) % leaves.Length)]);
                //    //tree.Insert(leaves[i]);
                //    //tree.InsertGlobal(leaves[i]);
                //}
                tree.BuildMedianSplit(leaves);
                //tree.BuildVolumeHeuristic(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline Build Time: {endTime - startTime}, depth: {tree.MaximumDepth}");

                Console.WriteLine($"Cost heuristic: {tree.MeasureCostHeuristic()}");

                tree.Validate();

                //var leafCount = tree.LeafCount;
                //for (int i = 0; i < leafCount; ++i)
                //{
                //    tree.RemoveAt(0);
                //    tree.Validate();
                //}


                int nodeCount, childCount;
                tree.MeasureNodeOccupancy(out nodeCount, out childCount);


                Console.WriteLine($"Baseline Occupancy: {childCount / (double)nodeCount}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //tree.RefitLeaves();
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline Refit Time: {endTime - startTime}");
                
                var list = new QuickList<int>(new BufferPool<int>());
                var queryMask = queries.Length - 1;
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < queryCount; ++i)
                {
                    list.Count = 0;
                    //tree.Query(ref queries[i & queryMask], ref list);
                    tree.QueryRecursive(ref queries[i & queryMask], ref list);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline Query Time: {endTime - startTime}, overlaps: {list.Count}");
                list.Dispose();

                var overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlaps(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline SelfTree Time: {endTime - startTime}, overlaps: {overlaps.Count}");

                overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlapsViaQueries(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline SelfQuery Time: {endTime - startTime}, overlaps: {overlaps.Count}");
            }
        }
    }
}
