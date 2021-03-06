﻿using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Engines;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Benchmarks2
{
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 8, invocationCount: 128, id: nameof(ShapePile2))]
    public class ShapePile2 : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Solver.IterationCount = 8;
            var box = new Box(1f, 3f, 2f);
            var capsule = new Capsule(1f, 1f);
            var sphere = new Sphere(1.5f);
            box.ComputeInertia(1, out var boxInertia);
            capsule.ComputeInertia(1, out var capsuleInertia);
            sphere.ComputeInertia(1, out var sphereInertia);
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            const int width = 16;
            const int height = 16;
            const int length = 16;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(3, 3, 3) * new Vector3(i, j, k) + new Vector3(-width * 1.5f, 1.5f, -length * 1.5f);
                        var bodyDescription = new BodyDescription
                        {
                            //Note no sleeping.
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0.00f },
                            Pose = new RigidPose
                            {
                                Orientation = BepuUtilities.Quaternion.Identity,
                                Position = location
                            },
                            Collidable = new CollidableDescription
                            {
                                Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                SpeculativeMargin = 0.1f
                            }
                        };
                        switch (j % 3)
                        {
                            case 0:
                                bodyDescription.Collidable.Shape = boxIndex;
                                bodyDescription.LocalInertia = boxInertia;
                                break;
                            case 1:
                                bodyDescription.Collidable.Shape = capsuleIndex;
                                bodyDescription.LocalInertia = capsuleInertia;
                                break;
                            case 2:
                                bodyDescription.Collidable.Shape = sphereIndex;
                                bodyDescription.LocalInertia = sphereInertia;
                                break;
                        }
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }

            var staticShape = new Box(200, 1, 200);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            var staticDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = staticShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose
                {
                    Position = new Vector3(1, -0.5f, 1),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            Simulation.Statics.Add(staticDescription);

            //Primarily interested in the more complex simulation that occurs after everything has collapsed.
            for (int i = 0; i < 256; ++i)
            {
                Simulation.Timestep(1 / 60f, threadDispatcher);
            }
        }
    }
}
