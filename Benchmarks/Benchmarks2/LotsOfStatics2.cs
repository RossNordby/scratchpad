using BenchmarkDotNet.Attributes.Exporters;
using BenchmarkDotNet.Attributes.Jobs;
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
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 8, invocationCount: 128, id: nameof(LotsOfStatics2))]
    public class LotsOfStatics2 : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Solver.IterationCount = 8;
            var shape = new Box(1, 1, 1);
            shape.ComputeInertia(1, out var sphereInertia);
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 64;
            const int height = 1;
            const int length = 64;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(1.4f, 1.1f, 1.4f) * new Vector3(i, j, k) + new Vector3(-width * 0.7f, 2.5f, -length * 0.7f);
                        var bodyDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 8, SleepThreshold = 0.01f },
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

                        bodyDescription.Collidable.Shape = shapeIndex;
                        bodyDescription.LocalInertia = sphereInertia;
                        Simulation.Bodies.Add(ref bodyDescription);

                    }
                }
            }

            var staticShape = new Box(1, 1, 1);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
            const int staticGridWidth = 96;
            const float staticSpacing = 1.2f;
            var gridOffset = -0.5f * staticGridWidth * staticSpacing;
            for (int i = 0; i < staticGridWidth; ++i)
            {
                for (int j = 0; j < staticGridWidth; ++j)
                {
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
                            Position = new Vector3(
                                gridOffset + i * staticSpacing,
                                -0.707f,
                                gridOffset + j * staticSpacing),
                            Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1 + i, i * j % 10, -10 + -j)), (i ^ j) * 0.5f * (MathHelper.PiOver4))
                        }
                    };
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

            for (int i = 0; i < 512; ++i)
            {
                Update();
            }
        }
    }
}
