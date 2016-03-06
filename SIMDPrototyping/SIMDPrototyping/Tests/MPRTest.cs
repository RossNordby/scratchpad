using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.CollisionTests.CollisionAlgorithms;
using BEPUutilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Tests
{
    class MPRTest
    {
        static ConvexHullShape BuildHull()
        {
            var size = new Vector3(1, 1, 1);
            var center = size * -0.5f;
            return new ConvexHullShape(new Vector3[8]
            {
                center + size * new Vector3(0, 0, 0),
                center + size * new Vector3(1, 0, 0),
                center + size * new Vector3(0, 1, 0),
                center + size * new Vector3(1, 1, 0),
                center + size * new Vector3(0, 0, 1),
                center + size * new Vector3(1, 0, 1),
                center + size * new Vector3(0, 1, 1),
                center + size * new Vector3(1, 1, 1)
            });
        }

        static Ray GetRandomRay(ref BoundingBox surface, Random random)
        {
            Ray ray;
            ray.Position = new Vector3((float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f);
            var length = ray.Position.Length();
            ray.Direction = ray.Position / -length;
            ray.Position = ray.Direction * -100;
            return ray;
        }

        public static void Test()
        {

            var f0 = BuildHull();
            f0.CollisionMargin = 0;


            //Generate spheres all around the central froxel in such a way that we know that they're not colliding.
            var froxelSphereSurface = new BoundingBox(new Vector3(-1.51f, -1.51f, -1.51f), new Vector3(1.51f, 1.51f, 1.51f));

            int testIterations = 1000;
            int innerIterations = 1000;
            Random random = new Random(5);
            long sphereFroxelSeparatedTicks = 0;
            SphereShape sphere = new SphereShape(1);
            for (int i = 0; i < testIterations; ++i)
            {
                var ray = GetRandomRay(ref froxelSphereSurface, random);
                float t;
                ray.Intersects(ref froxelSphereSurface, out t);
                var sphereTransform = new RigidTransform { Position = ray.Position + ray.Direction * t, Orientation = Quaternion.Identity };

                var start = Stopwatch.GetTimestamp();
                for (int j = 0; j < innerIterations; ++j)
                {
                    if (MPRToolbox.AreLocalShapesOverlapping(f0, sphere, ref sphereTransform))
                    {
                        Trace.Fail("By construction there can be no intersection!");
                    }
                }
                var end = Stopwatch.GetTimestamp();
                sphereFroxelSeparatedTicks += (end - start);
            }
            Console.WriteLine($"Sphere-froxel separated: {(1e6 * sphereFroxelSeparatedTicks) / (testIterations * innerIterations * Stopwatch.Frequency)}");

            //Do the same kind of test, but now with intersection.
            froxelSphereSurface = new BoundingBox(new Vector3(-0.5f, -0.5f, -0.5f), new Vector3(0.5f, 0.5f, 0.5f));

            long sphereFroxelIntersectingTicks = 0;
            for (int i = 0; i < testIterations; ++i)
            {
                var ray = GetRandomRay(ref froxelSphereSurface, random);
                float t;
                ray.Intersects(ref froxelSphereSurface, out t);
                var sphereTransform = new RigidTransform { Position = ray.Position + ray.Direction * (t - 0.99f), Orientation = Quaternion.Identity };

                var start = Stopwatch.GetTimestamp();
                for (int j = 0; j < innerIterations; ++j)
                {
                    if (!MPRToolbox.AreLocalShapesOverlapping(f0, sphere, ref sphereTransform))
                    {
                        Trace.Fail("By construction there can be no separation!");
                    }
                }
                var end = Stopwatch.GetTimestamp();
                sphereFroxelIntersectingTicks += (end - start);

            }
            Console.WriteLine($"Sphere-froxel intersecting: {(1e6 * sphereFroxelIntersectingTicks) / (testIterations * innerIterations * Stopwatch.Frequency)}");

            //Create a surface for the rays to hit such that every query froxel will be just outside of the central froxel.
            var froxelFroxelSurface = new BoundingBox(new Vector3(-1.01f, -1.01f, -1.01f), new Vector3(1.01f, 1.01f, 1.01f));

            var queryHull = BuildHull();
            queryHull.CollisionMargin = 0;
            long froxelFroxelSeparatedTicks = 0;
            for (int i = 0; i < testIterations; ++i)
            {
                var ray = GetRandomRay(ref froxelFroxelSurface, random);
                float t;
                ray.Intersects(ref froxelFroxelSurface, out t);

                var queryTransform = new RigidTransform(ray.Position + ray.Direction * t);

                var start = Stopwatch.GetTimestamp();
                for (int j = 0; j < innerIterations; ++j)
                {
                    if (MPRToolbox.AreLocalShapesOverlapping(f0, queryHull, ref queryTransform))
                    {
                        Trace.Fail("By construction there can be no intersection!");
                    }
                }
                var end = Stopwatch.GetTimestamp();
                froxelFroxelSeparatedTicks += (end - start);

            }
            Console.WriteLine($"Froxel-froxel separated: {(1e6 * froxelFroxelSeparatedTicks) / (testIterations * innerIterations * Stopwatch.Frequency)}");

            //Same thing as above, but now with slight intersection.
            froxelFroxelSurface = new BoundingBox(new Vector3(-.99f, -.99f, -.99f), new Vector3(0.99f, 0.99f, 0.99f));

            long froxelFroxelIntersectingTicks = 0;
            for (int i = 0; i < testIterations; ++i)
            {
                var ray = GetRandomRay(ref froxelFroxelSurface, random);
                float t;
                ray.Intersects(ref froxelFroxelSurface, out t);

                var queryTransform = new RigidTransform(ray.Position + ray.Direction * t);

                var start = Stopwatch.GetTimestamp();
                for (int j = 0; j < innerIterations; ++j)
                {
                    if (!MPRToolbox.AreLocalShapesOverlapping(f0, queryHull, ref queryTransform))
                    {
                        Trace.Fail("By construction there can be no separation!");
                    }
                }
                var end = Stopwatch.GetTimestamp();
                froxelFroxelIntersectingTicks += (end - start);

            }
            Console.WriteLine($"Froxel-froxel intersecting: {(1e6 * froxelFroxelIntersectingTicks) / (testIterations * innerIterations * Stopwatch.Frequency)}");

        }
    }
}
