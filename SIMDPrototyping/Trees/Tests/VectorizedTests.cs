using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using SIMDPrototyping.Trees.Vectorized;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.Tests
{
    partial class TreeTest
    {
        public static void TestVectorized(TestCollidable[] leaves, BoundingBox[] queries, int queryCount, int selfTestCount, int refitCount)
        {
            {
                var warmLeaves = GetLeaves(8, 8, 8, 10, 10);
                Tree<TestCollidable> tree = new Tree<TestCollidable>();
                for (int i = 0; i < warmLeaves.Length; ++i)
                {
                    tree.Insert(warmLeaves[i]);
                }
                Console.WriteLine($"Cachewarm Build: {tree.LeafCount}");

                tree.RefitLeaves();

                var list = new QuickList<int>(new BufferPool<int>());
                BoundingBox aabb = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1, 1, 1) };
                tree.Query(ref aabb, ref list);
                list.Dispose();

                var overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                tree.GetSelfOverlaps(ref overlaps);
                Console.WriteLine($"Warm overlaps: {overlaps.Count}");

                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                tree.GetSelfOverlapsViaQueries(ref overlaps);
                Console.WriteLine($"Warm overlaps: {overlaps.Count}");

                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                tree.GetSelfOverlapsViaStreamingQueries(ref overlaps);
                Console.WriteLine($"Warm overlaps: {overlaps.Count}");
            }


            {

                Tree<TestCollidable> tree = new Tree<TestCollidable>(leaves.Length, 32);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[(int)((982451653L * i) % leaves.Length)]);
                    //tree.Insert(leaves[i]);
                }
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Build Time: {endTime - startTime}, depth: {tree.MaximumDepth}");

                int nodeCount, childCount;
                tree.MeasureNodeOccupancy(out nodeCount, out childCount);

                Console.WriteLine($"Occupancy: {childCount / (double)nodeCount}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < refitCount; ++i)
                {
                    //tree.RefitLeaves();
                    tree.Refit();
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Refit Time: {endTime - startTime}");

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
                Console.WriteLine($"Query Time: {endTime - startTime}, overlaps: {list.Count}");
                list.Dispose();


                var overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlaps(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SelfTree Time: {endTime - startTime}, overlaps: {overlaps.Count}");

                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlapsViaQueries(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"SelfQuery Time: {endTime - startTime}, overlaps: {overlaps.Count}");

                overlaps = new QuickList<Overlap>(new BufferPool<Overlap>());
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < selfTestCount; ++i)
                {
                    overlaps.Count = 0;
                    tree.GetSelfOverlapsViaStreamingQueries(ref overlaps);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"StreamingSelfQuery Time: {endTime - startTime}, overlaps: {overlaps.Count}");
            }
        }
    }
}
