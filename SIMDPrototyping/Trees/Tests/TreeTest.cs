#define RANDOMLEAVES

using BEPUphysics.DataStructures;
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

using Tree = SIMDPrototyping.Trees.Vectorized.Tree<SIMDPrototyping.Trees.TestCollidable>;
using BaselineTree = SIMDPrototyping.Trees.Baseline.Tree<SIMDPrototyping.Trees.TestCollidable>;
using SIMDPrototyping.Trees.Baseline;

namespace SIMDPrototyping.Trees.Tests
{

    public static class TreeTest
    {
        static TestCollidableBEPU[] GetRandomLeavesBEPU(int leafCount, BoundingBox bounds, Vector3 leafSize)
        {
            var leaves = new TestCollidableBEPU[leafCount];
            Random random = new Random(5);

            var range = bounds.Max - bounds.Min;
            for (int i = 0; i < leafCount; ++i)
            {
                var min = bounds.Min + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * range;
                var max = min + leafSize;

                leaves[i] = new TestCollidableBEPU();
                leaves[i].BoundingBox = new BEPUutilities.BoundingBox(new BEPUutilities.Vector3(min.X, min.Y, min.Z), new BEPUutilities.Vector3(max.X, max.Y, max.Z));
            }
            return leaves;

        }

        static TestCollidable[] GetRandomLeaves(int leafCount, BoundingBox bounds, Vector3 leafSize)
        {
            var leaves = new TestCollidable[leafCount];
            Random random = new Random(5);

            var range = bounds.Max - bounds.Min;
            for (int i = 0; i < leafCount; ++i)
            {
                leaves[i] = new TestCollidable();
                leaves[i].BoundingBox.Min = bounds.Min + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * range;
                leaves[i].BoundingBox.Max = leaves[i].BoundingBox.Min + leafSize;

            }
            return leaves;

        }
        static TestCollidableBEPU[] GetLeavesBEPU(int width, int height, int length, float size, float gap)
        {
            var leaves = new TestCollidableBEPU[width * height * length];
            var offset = size + gap;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var collidable = new TestCollidableBEPU();
                        BEPUutilities.BoundingBox boundingBox;
                        boundingBox.Min = new BEPUutilities.Vector3(i * offset, j * offset, k * offset);
                        boundingBox.Max = boundingBox.Min + new BEPUutilities.Vector3(size, size, size);
                        collidable.BoundingBox = boundingBox;
                        leaves[height * length * i + length * j + k] = collidable;
                    }
                }
            }
            return leaves;
        }
        static TestCollidable[] GetLeaves(int width, int height, int length, float size, float gap)
        {
            var leaves = new TestCollidable[width * height * length];
            var offset = size + gap;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var collidable = new TestCollidable();
                        collidable.BoundingBox.Min = new Vector3(i * offset, j * offset, k * offset);
                        collidable.BoundingBox.Max = collidable.BoundingBox.Min + new Vector3(size);
                        leaves[height * length * i + length * j + k] = collidable;
                    }
                }
            }
            return leaves;


        }

        static BoundingBox[] GetQueryLocations(int count, BoundingBox bounds, Vector3 size)
        {
            Random random = new Random(5);
            BoundingBox[] boxes = new BoundingBox[count];
            var range = bounds.Max - bounds.Min;
            for (int i = 0; i < boxes.Length; ++i)
            {
                var x = (float)random.NextDouble();
                var y = (float)random.NextDouble();
                var z = (float)random.NextDouble();
                boxes[i].Min = bounds.Min + new Vector3(x, y, z) * range;
                boxes[i].Max = boxes[i].Min + size;
            }
            return boxes;
        }

        static BEPUutilities.BoundingBox[] GetBEPUQueryLocations(int count, BoundingBox bounds, Vector3 size)
        {
            Random random = new Random(5);
            BEPUutilities.BoundingBox[] boxes = new BEPUutilities.BoundingBox[count];
            var range = bounds.Max - bounds.Min;
            for (int i = 0; i < boxes.Length; ++i)
            {

                boxes[i].Min = new BEPUutilities.Vector3(
                    bounds.Min.X + ((float)random.NextDouble()) * range.X,
                    bounds.Min.Y + ((float)random.NextDouble()) * range.Y,
                    bounds.Min.Z + ((float)random.NextDouble()) * range.Z);
                boxes[i].Max = boxes[i].Min + new BEPUutilities.Vector3(size.X, size.Y, size.Z);
            }
            return boxes;
        }
        public static void Test()
        {
            GC.Collect();
            {
                var leaves = GetLeaves(64, 64, 64, 10, 10);
                Tree tree = new Tree();
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[i]);
                }
                Console.WriteLine($"Cachewarm Build: {tree.LeafCount}");

                tree.Refit();

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

            float leafSize = 10;
            int queryCount = 100000;
            int selfTestCount = 10;
#if RANDOMLEAVES
            BoundingBox randomLeafBounds = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1000, 1000, 1000) };
            BoundingBox queryBounds = randomLeafBounds;
            int randomLeafCount = 262144;
#else
            int leafCountX = 64;
            int leafCountY = 64;
            int leafCountZ = 64;
            float leafGap = 10;
            BoundingBox queryBounds = new BoundingBox { Min = new Vector3(0), Max = new Vector3(leafCountX, leafCountY, leafCountZ) * (new Vector3(leafSize) + new Vector3(leafGap)) };
#endif

            Vector3 querySize = new Vector3(20);
            int queryLocationCount = 16384;
            int queryMask = queryLocationCount - 1;
            {

#if RANDOMLEAVES
                var leaves = GetRandomLeaves(randomLeafCount, randomLeafBounds, new Vector3(leafSize));
#else
                var leaves = GetLeaves(leafCountX, leafCountY, leafCountZ, leafSize, leafGap);
#endif
                Tree tree = new Tree(leaves.Length, 32);
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
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Refit Time: {endTime - startTime}");

                var queries = GetQueryLocations(queryLocationCount, queryBounds, querySize);
                var list = new QuickList<int>(new BufferPool<int>());
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

            GC.Collect();


            GC.Collect();
            {
                var leaves = GetLeaves(10, 10, 10, 10, 10);
                BaselineTree tree = new BaselineTree();
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[i]);
                }
                //tree.BuildMedianSplit(leaves);
                //tree.BuildVolumeHeuristic(leaves);
                Console.WriteLine($"Baseline Cachewarm Build: {tree.LeafCount}");

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
#if RANDOMLEAVES
                var leaves = GetRandomLeaves(randomLeafCount, randomLeafBounds, new Vector3(leafSize));
#else
                var leaves = GetLeaves(leafCountX, leafCountY, leafCountZ, leafSize, leafGap);
#endif
                BaselineTree tree = new BaselineTree(leaves.Length, 32);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[(int)((982451653L * i) % leaves.Length)]);
                    //tree.Insert(leaves[i]);
                }
                //tree.BuildMedianSplit(leaves);
                //tree.BuildVolumeHeuristic(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline Build Time: {endTime - startTime}, depth: {tree.MaximumDepth}");

                Console.WriteLine($"Cost heuristic: {tree.MeasureCostHeuristic()}");

                int nodeCount, childCount;
                tree.MeasureNodeOccupancy(out nodeCount, out childCount);

                Console.WriteLine($"Baseline Occupancy: {childCount / (double)nodeCount}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline Refit Time: {endTime - startTime}");

                var queries = GetQueryLocations(queryLocationCount, queryBounds, querySize);
                var list = new QuickList<int>(new BufferPool<int>());
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


            GC.Collect();
            {
                var leaves = GetLeavesBEPU(2, 2, 2, 10, 10);
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                Console.WriteLine($"BEPU Cachewarm Build, root AABB: {tree.BoundingBox}");

                tree.Refit();

                RawList<TestCollidableBEPU> results = new RawList<TestCollidableBEPU>();
                BEPUutilities.BoundingBox aabb = new BEPUutilities.BoundingBox { Min = new BEPUutilities.Vector3(0, 0, 0), Max = new BEPUutilities.Vector3(1, 1, 1) };

                results.Count = 0;
                tree.GetOverlaps(aabb, results);
            }

            {

#if RANDOMLEAVES
                var leaves = GetRandomLeavesBEPU(randomLeafCount, randomLeafBounds, new Vector3(leafSize));
#else
                var leaves = GetLeavesBEPU(leafCountX, leafCountY, leafCountZ, leafSize, leafGap);
#endif
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Build Time: {endTime - startTime}, root AABB: {tree.BoundingBox}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Refit Time: {endTime - startTime}");

                var queries = GetBEPUQueryLocations(queryLocationCount, queryBounds, querySize);
                RawList<TestCollidableBEPU> results = new RawList<TestCollidableBEPU>();
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < queryCount; ++i)
                {
                    results.Count = 0;
                    tree.GetOverlaps(queries[i & queryMask], results);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Query Time: {endTime - startTime}, overlaps: {results.Count}");

            }

        }
    }
}

