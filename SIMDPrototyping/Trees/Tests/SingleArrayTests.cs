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
        public static void TestSingleArray(TestCollidable[] leaves, BoundingBox[] queries, int queryCount, int selfTestCount)
        {
            {
                var warmLeaves = GetLeaves(10, 10, 10, 10, 10);
                Tree tree = new Tree();
                //for (int i = 0; i < leaves.Length; ++i)
                //{
                //    BoundingBox box;
                //    leaves[i].GetBoundingBox(out box);
                //    tree.Insert(i, ref box);
                //}
                int[] leafIds = new int[warmLeaves.Length];
                BoundingBox[] leafBounds = new BoundingBox[warmLeaves.Length];
                for (int i = 0; i < warmLeaves.Length; ++i)
                {
                    leafIds[i] = i;
                    warmLeaves[i].GetBoundingBox(out leafBounds[i]);
                }
                tree.BuildMedianSplit(leafIds, leafBounds);
                //tree.BuildVolumeHeuristic(warmLeaves);
                Console.WriteLine($"SingleArray Cachewarm Build: {tree.LeafCount}");

                tree.Refit();

                var list = new QuickList<int>(new BufferPool<int>());
                BoundingBox aabb = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1, 1, 1) };
                tree.QueryRecursive(ref aabb, ref list);
                list.Dispose();

                //var overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());
                //tree.GetSelfOverlaps(ref overlaps);
                //Console.WriteLine($"Cachewarm overlaps: {overlaps.Count}");

                //overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());

                //tree.GetSelfOverlapsViaQueries(ref overlaps);
                //Console.WriteLine($"Cachewarm overlaps: {overlaps.Count}");
            }

            {
                Console.WriteLine($"SingleArray arity: {Tree.ChildrenCapacity}");
                Tree tree = new Tree(leaves.Length);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //for (int i = 0; i < leaves.Length; ++i)
                //{
                //    var leafIndex = (int)((982451653L * i) % leaves.Length);
                //    BoundingBox box;
                //    leaves[i].GetBoundingBox(out box);
                //    tree.Insert(leafIndex, ref box);
                //    //tree.InsertGlobal(leaves[(int)((982451653L * i) % leaves.Length)]);
                //    //tree.Insert(leaves[i]);
                //    //tree.InsertGlobal(leaves[i]);
                //}
                int[] leafIds = new int[leaves.Length];
                BoundingBox[] leafBounds = new BoundingBox[leaves.Length];
                for (int i = 0; i < leaves.Length; ++i)
                {
                    leafIds[i] = i;
                    leaves[i].GetBoundingBox(out leafBounds[i]);
                }
                tree.BuildMedianSplit(leafIds, leafBounds);
                //tree.BuildVolumeHeuristic(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Build Time: {endTime - startTime}, depth: {tree.ComputeMaximumDepth()}");

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


                Console.WriteLine($"SingleArray Occupancy: {childCount / (double)nodeCount}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //for (int i = 0; i < tree.LeafCount; ++i)
                //{
                //    BoundingBox box;
                //    leaves[tree.Leaves[i].Id].GetBoundingBox(out box);
                //    tree.UpdateLeafBoundingBox(i, ref box);
                //}
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SingleArray Refit Time: {endTime - startTime}");

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
                Console.WriteLine($"SingleArray Query Time: {endTime - startTime}, overlaps: {list.Count}");
                list.Dispose();

                //var overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());
                //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //for (int i = 0; i < selfTestCount; ++i)
                //{
                //    overlaps.Count = 0;
                //    tree.GetSelfOverlaps(ref overlaps);
                //}
                //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //Console.WriteLine($"SingleArray SelfTree Time: {endTime - startTime}, overlaps: {overlaps.Count}");

                //overlaps = new QuickList<Overlap<TestCollidable>>(new BufferPool<Overlap<TestCollidable>>());
                //startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //for (int i = 0; i < selfTestCount; ++i)
                //{
                //    overlaps.Count = 0;
                //    tree.GetSelfOverlapsViaQueries(ref overlaps);
                //}
                //endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                //Console.WriteLine($"SingleArray SelfQuery Time: {endTime - startTime}, overlaps: {overlaps.Count}");
            }
        }
    }
}
