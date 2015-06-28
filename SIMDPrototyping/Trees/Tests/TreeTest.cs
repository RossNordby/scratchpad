using BEPUphysics.DataStructures;
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
                        boundingBox.Max = collidable.BoundingBox.Min + new BEPUutilities.Vector3(size, size, size);
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
            {
                var leaves = GetLeaves(2, 2, 2, 10, 10);
                Tree<TestCollidable> tree = new Tree<TestCollidable>();
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[i]);
                }
                Console.WriteLine($"Cachewarm: {tree.LeafCount}");
            }

            {
                var leaves = GetLeaves(16, 16, 16, 10, 10);
                Tree<TestCollidable> tree = new Tree<TestCollidable>();
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                for (int i = 0; i < leaves.Length; ++i)
                {
                    tree.Insert(leaves[i]);
                }
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Time: {endTime - startTime}, depth: {tree.MaximumDepth}");
            }

            {
                var leaves = GetLeavesBEPU(16, 16, 16, 10, 10);
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                Console.WriteLine($"BEPU Cachewarm root AABB: {tree.BoundingBox}");
            }
            {

                var leaves = GetLeavesBEPU(16, 16, 16, 10, 10);
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                BoundingBoxTree<TestCollidableBEPU> tree = new BoundingBoxTree<TestCollidableBEPU>(leaves);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"BEPU Time: {endTime - startTime}, root AABB: {tree.BoundingBox}");
            }

        }
    }
}

