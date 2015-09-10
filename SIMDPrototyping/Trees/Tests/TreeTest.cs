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
using System.IO;

namespace SIMDPrototyping.Trees.Tests
{

    public static partial class TreeTest
    {

        struct VelocityDescription
        {
            public float MinVelocity;
            public float MaxVelocity;
            public float VelocityDistributionPower;
            public float PortionOfMovingLeaves;
        }

        static TestCollidableBEPU[] GetRandomLeavesBEPU(int leafCount, BoundingBox bounds, Vector3 minimumSize, Vector3 maximumSize, float sizePower, VelocityDescription velocityDescription)
        {
            var leaves = new TestCollidableBEPU[leafCount];
            Random random = new Random(5);

            var range = bounds.Max - bounds.Min;
            var sizeRange = maximumSize - minimumSize;
            for (int i = 0; i < leafCount; ++i)
            {
                var halfLeafSize = 0.5f * (minimumSize + new Vector3((float)Math.Pow(random.NextDouble(), sizePower), (float)Math.Pow(random.NextDouble(), sizePower), (float)Math.Pow(random.NextDouble(), sizePower)) * sizeRange);
                var position = bounds.Min + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * range;

                leaves[i] = new TestCollidableBEPU();
                leaves[i].Position = new BEPUutilities.Vector3(position.X, position.Y, position.Z);
                leaves[i].HalfSize = new BEPUutilities.Vector3(halfLeafSize.X, halfLeafSize.Y, halfLeafSize.Z);
                leaves[i].UpdateBoundingBox();
            }

            for (int i = 0; i < leaves.Length * velocityDescription.PortionOfMovingLeaves; ++i)
            {
                var speed = (float)(velocityDescription.MinVelocity + (velocityDescription.MaxVelocity - velocityDescription.MinVelocity) * Math.Pow(random.NextDouble(), velocityDescription.VelocityDistributionPower));
                var direction = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * 2 - Vector3.One;
                var lengthSquared = direction.LengthSquared();
                if (lengthSquared < 1e-9f)
                {
                    direction = new Vector3(0, 1, 0);
                }
                else
                {
                    direction /= (float)Math.Sqrt(lengthSquared);
                }
                var velocity = speed * direction;
                leaves[i].Velocity = new BEPUutilities.Vector3(velocity.X, velocity.Y, velocity.Z);
            }
            return leaves;

        }

        static TestCollidable[] GetRandomLeaves(int leafCount, BoundingBox bounds, Vector3 minimumSize, Vector3 maximumSize, float sizePower, VelocityDescription velocityDescription)
        {
            var leaves = new TestCollidable[leafCount];
            Random random = new Random(5);

            var range = bounds.Max - bounds.Min;
            var sizeRange = maximumSize - minimumSize;
            for (int i = 0; i < leafCount; ++i)
            {
                leaves[i] = new TestCollidable();
                leaves[i].HalfSize = 0.5f * (minimumSize + new Vector3((float)Math.Pow(random.NextDouble(), sizePower), (float)Math.Pow(random.NextDouble(), sizePower), (float)Math.Pow(random.NextDouble(), sizePower)) * sizeRange);
                leaves[i].Position = bounds.Min + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * range;

            }

            for (int i = 0; i < leaves.Length * velocityDescription.PortionOfMovingLeaves; ++i)
            {
                var speed = (float)(velocityDescription.MinVelocity + (velocityDescription.MaxVelocity - velocityDescription.MinVelocity) * Math.Pow(random.NextDouble(), velocityDescription.VelocityDistributionPower));
                var direction = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * 2 - Vector3.One;
                var lengthSquared = direction.LengthSquared();
                if (lengthSquared < 1e-9f)
                {
                    direction = new Vector3(0, 1, 0);
                }
                else
                {
                    direction /= (float)Math.Sqrt(lengthSquared);
                }
                leaves[i].Velocity = speed * direction;
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
                        collidable.HalfSize = new Vector3(size * 0.5f);
                        collidable.Position = collidable.HalfSize + new Vector3(i * offset, j * offset, k * offset);
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
            float leafMinSize = 1;
            float leafMaxSize = 100;
            float leafSizePower = 10;
            int queryCount = 1000000;
            int selfTestCount = 10;
            int refitCount = 100;
            int frameCount = 2048;
            float dt = 1 / 60f;

            VelocityDescription velocityDescription = new VelocityDescription
            {
                MinVelocity = 0,
                MaxVelocity = 10,
                VelocityDistributionPower = 10,
                PortionOfMovingLeaves = 1
            };

            Vector3 querySize = new Vector3(20);
            int queryLocationCount = 16384; //<-- POWER OF TWO!!! REMEMBER!


#if RANDOMLEAVES
            BoundingBox randomLeafBounds = new BoundingBox { Min = new Vector3(0, 0, 0), Max = new Vector3(629.96f) };
            BoundingBox queryBounds = randomLeafBounds;
            int randomLeafCount = 16384;

#else
            int leafCountX = 64;
            int leafCountY = 64;
            int leafCountZ = 64;
            float leafGap = 10;
            BoundingBox queryBounds = new BoundingBox { Min = new Vector3(0), Max = new Vector3(leafCountX, leafCountY, leafCountZ) * (new Vector3(leafSize) + new Vector3(leafGap)) };
#endif

            {

                var queries = GetQueryLocations(queryLocationCount, queryBounds, querySize);

#if RANDOMLEAVES
                var leaves = GetRandomLeaves(randomLeafCount, randomLeafBounds, new Vector3(leafMinSize), new Vector3(leafMaxSize), leafSizePower, velocityDescription);
#else
                var leaves = GetLeaves(leafCountX, leafCountY, leafCountZ, leafSize, leafGap);
#endif
                GC.Collect();
                //TestVectorized(leaves, queries, queryCount, selfTestCount, refitCount);
#if RANDOMLEAVES
                leaves = GetRandomLeaves(randomLeafCount, randomLeafBounds, new Vector3(leafMinSize), new Vector3(leafMaxSize), leafSizePower, velocityDescription);
#else
                leaves = GetLeaves(leafCountX, leafCountY, leafCountZ, leafSize, leafGap);
#endif
                GC.Collect();
                //TestBaseline(leaves, queries, queryCount, selfTestCount, refitCount);
#if RANDOMLEAVES
                leaves = GetRandomLeaves(randomLeafCount, randomLeafBounds, new Vector3(leafMinSize), new Vector3(leafMaxSize), leafSizePower, velocityDescription);
#else
                leaves = GetLeaves(leafCountX, leafCountY, leafCountZ, leafSize, leafGap);
#endif
                GC.Collect();
                var results = TestSingleArray(leaves, queries, randomLeafBounds, queryCount, selfTestCount, refitCount, frameCount, dt);

                using (var stream = File.Open("newTreeResults.txt", FileMode.Create))
                {
                    using (var textWriter = new StreamWriter(stream))
                    {
                        results.Save(textWriter);
                    }
                }
            }

            {

#if RANDOMLEAVES
                var leaves = GetRandomLeavesBEPU(randomLeafCount, randomLeafBounds, new Vector3(leafMinSize), new Vector3(leafMaxSize), leafSizePower, velocityDescription);
#else
                var leaves = GetLeavesBEPU(leafCountX, leafCountY, leafCountZ, leafSize, leafGap);
#endif
                var queries = GetBEPUQueryLocations(queryLocationCount, queryBounds, querySize);

                GC.Collect();
                //TestBEPU(leaves, queries, queryCount, selfTestCount, refitCount);
                //var results = TestDH(leaves, queries, ref randomLeafBounds, queryCount, selfTestCount, refitCount, frameCount, dt);
                //using (var stream = File.Open("oldDHResults.txt", FileMode.Create))
                //{
                //    using (var textWriter = new StreamWriter(stream))
                //    {
                //        results.Save(textWriter);
                //    }
                //}
            }

        }
    }
}

