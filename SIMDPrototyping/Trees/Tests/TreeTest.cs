using BEPUphysics.DataStructures;
using BEPUutilities.DataStructures;
using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

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
                        leaves[width * height * i + height * j + k] = collidable;
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
                        leaves[width * height * i + height * j + k] = collidable;
                    }
                }
            }
            return leaves;


        }
        public static void Test()
        {
            GC.Collect();
            {
                var leaves = GetLeaves(2, 2, 2, 10, 10);
                Tree<TestCollidable> tree = new Tree<TestCollidable>();
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

            int queryCount = 1;
            {
                var leaves = GetLeaves(64, 64, 64, 10, 10);
                Tree<TestCollidable> tree = new Tree<TestCollidable>();
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[(int)((982451653L * i) % leaves.Length)]);
                    //tree.Insert(leaves[i]);
                }
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Build Time: {endTime - startTime}, depth: {tree.MaximumDepth}");


                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Refit Time: {endTime - startTime}");

                var list = new QuickList<TestCollidable>(new BufferPool<TestCollidable>());
                BoundingBox aabb = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(1, 1, 1) };
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < queryCount; ++i)
                {
                    list.Count = 0;
                    tree.Query(ref aabb, ref list);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Query Time: {endTime - startTime}, overlaps: {list.Count}");
                list.Dispose();
            }

            GC.Collect();
            {
                var leaves = GetLeavesBEPU(2, 2, 2, 10, 10);
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                Console.WriteLine($"BEPU Cachewarm Build, root AABB: {tree.BoundingBox}");
                
                tree.Refit();
            }
            {

                var leaves = GetLeavesBEPU(64, 64, 64, 10, 10);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Build Time: {endTime - startTime}, root AABB: {tree.BoundingBox}");

                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                tree.Refit();
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Refit Time: {endTime - startTime}");

                RawList<TestCollidableBEPU> results = new RawList<TestCollidableBEPU>();
                BEPUutilities.BoundingBox aabb = new BEPUutilities.BoundingBox { Min = new BEPUutilities.Vector3(0, 0, 0), Max = new BEPUutilities.Vector3(1, 1, 1) };
                startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < queryCount; ++i)
                {
                    results.Count = 0;
                    tree.GetOverlaps(aabb, results);
                }
                endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Query Time: {endTime - startTime}, overlaps: {results.Count}");

            }

        }
    }
}

