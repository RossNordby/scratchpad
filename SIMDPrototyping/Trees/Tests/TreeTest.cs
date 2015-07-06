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

namespace SIMDPrototyping.Trees.Tests
{

    public static class TreeTest
    {
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

        static BoundingBox[] GetQueryLocations(int count, float range, float size)
        {
            Random random = new Random(5);
            BoundingBox[] boxes = new BoundingBox[count];
            for (int i = 0; i < boxes.Length; ++i)
            {
                boxes[i].Min = new Vector3((float)(random.NextDouble() * range), (float)(random.NextDouble() * range), (float)(random.NextDouble() * range));
                boxes[i].Max = boxes[i].Min + new Vector3(size);
            }
            return boxes;
        }

        static BEPUutilities.BoundingBox[] GetBEPUQueryLocations(int count, float range, float size)
        {
            Random random = new Random(5);
            BEPUutilities.BoundingBox[] boxes = new BEPUutilities.BoundingBox[count];
            for (int i = 0; i < boxes.Length; ++i)
            {
                boxes[i].Min = new BEPUutilities.Vector3((float)(random.NextDouble() * range), (float)(random.NextDouble() * range), (float)(random.NextDouble() * range));
                boxes[i].Max = boxes[i].Min + new BEPUutilities.Vector3(size, size, size);
            }
            return boxes;
        }
        public static void Test()
        {
            GC.Collect();
            {
                var leaves = GetLeaves(2, 2, 2, 10, 10);
                Tree tree = new Tree();
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[i]);
                }
                Console.WriteLine($"Cachewarm Build: {tree.LeafCount}");

                tree.Refit();

                var list = new QuickList<TestCollidable>(new BufferPool<TestCollidable>());
                BoundingBox aabb = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1, 1, 1) };
                tree.Query(ref aabb, ref list);
                list.Dispose();
            }

            int leafCubeSize = 64;
            float leafSize = 10, leafGap = 10f;
            int queryCount = 10000000;
            float queryRange = leafCubeSize * (leafSize + leafGap), querySize = 0;
            int queryLocationCount = 4096;
            int queryMask = queryLocationCount - 1;
            {
                
                var leaves = GetLeaves(leafCubeSize, leafCubeSize, leafCubeSize, leafSize, leafGap);
                Tree tree = new Tree(262144, 32);
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

                var queries = GetQueryLocations(queryLocationCount, queryRange, querySize);
                var list = new QuickList<TestCollidable>(new BufferPool<TestCollidable>());
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
            }

            GC.Collect();


            GC.Collect();
            {
                var leaves = GetLeaves(2, 2, 2, 10, 10);
                BaselineTree tree = new BaselineTree();
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[i]);
                }
                Console.WriteLine($"Baseline Cachewarm Build: {tree.LeafCount}");

                tree.Refit();

                var list = new QuickList<TestCollidable>(new BufferPool<TestCollidable>());
                BoundingBox aabb = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1, 1, 1) };
                tree.Query(ref aabb, ref list);
                list.Dispose();
            }

            {
                Console.WriteLine($"Baseline arity: {BaselineTree.ChildrenCapacity}");
                var leaves = GetLeaves(leafCubeSize, leafCubeSize, leafCubeSize, leafSize, leafGap);
                BaselineTree tree = new BaselineTree(262144, 32);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[(int)((982451653L * i) % leaves.Length)]);
                    //tree.Insert(leaves[i]);
                }
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline Build Time: {endTime - startTime}, depth: {tree.MaximumDepth}");

                int nodeCount, childCount;
                tree.MeasureNodeOccupancy(out nodeCount, out childCount);

                Console.WriteLine($"Baseline Occupancy: {childCount / (double)nodeCount}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Baseline Refit Time: {endTime - startTime}");

                var queries = GetQueryLocations(queryLocationCount, queryRange, querySize);
                var list = new QuickList<TestCollidable>(new BufferPool<TestCollidable>());
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

                var leaves = GetLeavesBEPU(leafCubeSize, leafCubeSize, leafCubeSize, leafSize, leafGap);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Build Time: {endTime - startTime}, root AABB: {tree.BoundingBox}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Refit Time: {endTime - startTime}");

                var queries = GetBEPUQueryLocations(queryLocationCount, queryRange, querySize);
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

