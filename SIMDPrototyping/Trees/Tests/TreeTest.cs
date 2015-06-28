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
        static Tree<TestCollidable> MakeTree(int width, int height, int length, float size, float gap)
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
            Tree<TestCollidable> tree = new Tree<TestCollidable>();
            for (int i = 0; i < leaves.Length; ++i)
            {
                tree.Insert(leaves[i]);
            }
            return tree;

        }
        public static void Test()
        {
            {
                var tree = MakeTree(2, 2, 2, 10, 10);
                Console.WriteLine($"Cachewarm: {tree.LeafCount}");
            }

            {
                var startTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                var tree = MakeTree(16, 16, 16, 10, 10);
                var endTime = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
                Console.WriteLine($"Time: {endTime - startTime}, depth: {tree.MaximumDepth}");
            }

        }
    }
}

