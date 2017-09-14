﻿using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
using SolverPrototype.CollisionDetection.CollisionTasks;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototypeTests.SpecializedTests
{
    public static class BatchedCollisionTests
    {
        struct ContinuationsTest : IContinuations
        {
            public int Count;
            public unsafe void Notify(TypedIndex continuationId, ref ContactManifold manifold)
            {
                Console.WriteLine($"Completed {continuationId}:");
                var local = manifold;
                var normals = &local.Normal0;
                var offsets = &local.Offset0;
                var depths = &local.Depth0;
                if (manifold.Convex)
                {
                    for (int i = 0; i < manifold.ContactCount; ++i)
                    {
                        Console.WriteLine($"{i}: P: {offsets[i]}, N: {manifold.ConvexNormal}, D: {depths[i]}");
                    }
                }
                else
                {
                    for (int i = 0; i < manifold.ContactCount; ++i)
                    {
                        Console.WriteLine($"{i}: P: {offsets[i]}, N: {normals[i]}, D: {depths[i]}");
                    }
                }
                var extra = 1e-16 * (manifold.Depth0 + manifold.Offset0.X + manifold.Normal0.X);
                Count += 1 + (int)extra;
            }
        }
        struct SubtaskFiltersTest : ICollisionSubtaskFilters
        {
            public bool AllowCollisionTesting<TInput>(ref TInput input) where TInput : struct
            {
                return true;
            }

            public void Configure<TInput>(ref TInput input) where TInput : struct
            {
            }
        }

        static double Test(Action<int> action, int iterationCount)
        {
            action(64);

            var start = Stopwatch.GetTimestamp();
            action(iterationCount);
            var end = Stopwatch.GetTimestamp();
            return (end - start) / (double)Stopwatch.Frequency;
        }

        public static void Test()
        {
            DefaultTypes.Register();
            var pool = new BufferPool();
            var registry = new CollisionTaskRegistry();
            var task = new SpherePairCollisionTask();
            registry.Register(task);
            var continuations = new ContinuationsTest();
            var filters = new SubtaskFiltersTest();
            var sphere = new Sphere(1);
            var poseA = new BodyPose { Position = new Vector3(0, 0, 0), Orientation = BEPUutilities2.Quaternion.Identity };
            var poseB = new BodyPose { Position = new Vector3(0, 1, 0), Orientation = BEPUutilities2.Quaternion.Identity };
            Action<int> action = iterationCount =>
            {
                var batcher = new StreamingBatcher(pool, registry);
                for (int i = 0; i < iterationCount; ++i)
                {
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new TypedIndex(), ref continuations, ref filters);
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new TypedIndex(), ref continuations, ref filters);
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new TypedIndex(), ref continuations, ref filters);
                    batcher.Add(ref sphere, ref sphere, ref poseA, ref poseB, new TypedIndex(), ref continuations, ref filters);
                }
                batcher.Flush(ref continuations, ref filters);
            };
            var time0 = Test(action, 1 << 2);
            Console.WriteLine($"Completed count: {continuations.Count}, time (ms): {1e3 * time0}");
            Console.ReadKey();
        }
    }
}