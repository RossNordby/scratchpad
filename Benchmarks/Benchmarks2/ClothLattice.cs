﻿using BenchmarkDotNet.Attributes.Jobs;
using BenchmarkDotNet.Engines;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using System.Numerics;

namespace Benchmarks2
{
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 8, invocationCount: 128)]
    public class ClothLattice : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            //Build a grid of shapes to be connected.
            var clothNodeShape = new Sphere(0.5f);
            clothNodeShape.ComputeInertia(1, out var clothNodeInertia);
            var clothNodeShapeIndex = Simulation.Shapes.Add(ref clothNodeShape);
            const int width = 128;
            const int length = 128;
            const float spacing = 1.75f;
            int[][] nodeHandles = new int[width][];
            for (int i = 0; i < width; ++i)
            {
                nodeHandles[i] = new int[length];
                for (int j = 0; j < length; ++j)
                {
                    var location = new Vector3(0, 30, 0) + new Vector3(spacing, 0, spacing) * (new Vector3(i, 0, j) + new Vector3(-width * 0.5f, 0, -length * 0.5f));
                    var bodyDescription = new BodyDescription
                    {
                        Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0.0f },
                        Pose = new RigidPose
                        {
                            Orientation = BepuUtilities.Quaternion.Identity,
                            Position = location
                        },
                        Collidable = new CollidableDescription
                        {
                            Shape = clothNodeShapeIndex,
                            Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                            SpeculativeMargin = 0.1f
                        },
                        LocalInertia = clothNodeInertia
                    };
                    nodeHandles[i][j] = Simulation.Bodies.Add(ref bodyDescription);

                }
            }
            //Construct some joints between the nodes.
            var left = new BallSocket
            {
                LocalOffsetA = new Vector3(-spacing * 0.5f, 0, 0),
                LocalOffsetB = new Vector3(spacing * 0.5f, 0, 0),
                SpringSettings = new SpringSettings(10, 1)
            };
            var up = new BallSocket
            {
                LocalOffsetA = new Vector3(0, 0, -spacing * 0.5f),
                LocalOffsetB = new Vector3(0, 0, spacing * 0.5f),
                SpringSettings = new SpringSettings(10, 1)
            };
            var leftUp = new BallSocket
            {
                LocalOffsetA = new Vector3(-spacing * 0.5f, 0, -spacing * 0.5f),
                LocalOffsetB = new Vector3(spacing * 0.5f, 0, spacing * 0.5f),
                SpringSettings = new SpringSettings(10, 1)
            };
            var rightUp = new BallSocket
            {
                LocalOffsetA = new Vector3(spacing * 0.5f, 0, -spacing * 0.5f),
                LocalOffsetB = new Vector3(-spacing * 0.5f, 0, spacing * 0.5f),
                SpringSettings = new SpringSettings(10, 1)
            };
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < length; ++j)
                {
                    if (i >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i - 1][j], ref left);
                    if (j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i][j - 1], ref up);
                    if (i >= 1 && j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i - 1][j - 1], ref leftUp);
                    if (i < width - 1 && j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i + 1][j - 1], ref rightUp);
                }
            }
            var bigBallShape = new Sphere(25);
            var bigBallShapeIndex = Simulation.Shapes.Add(ref bigBallShape);

            var bigBallDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = bigBallShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose
                {
                    Position = new Vector3(-10, -15, 0),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            Simulation.Statics.Add(ref bigBallDescription);

            var groundShape = new Box(200, 1, 200);
            var groundShapeIndex = Simulation.Shapes.Add(ref groundShape);

            var groundDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = groundShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose
                {
                    Position = new Vector3(0, -10, 0),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            Simulation.Statics.Add(ref groundDescription);
        }
    }
}
