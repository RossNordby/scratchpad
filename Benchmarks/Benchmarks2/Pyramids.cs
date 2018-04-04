using BenchmarkDotNet.Attributes.Jobs;
using BenchmarkDotNet.Engines;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;

namespace Benchmarks2
{
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 8, invocationCount: 64, id: nameof(Pyramids) + " v2")]
    public class Pyramids : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Solver.IterationCount = 8;

            var boxShape = new Box(1, 1, 1);
            boxShape.ComputeInertia(1, out var boxInertia);
            var boxIndex = Simulation.Shapes.Add(ref boxShape);
            const int pyramidCount = 20;
            for (int pyramidIndex = 0; pyramidIndex < pyramidCount; ++pyramidIndex)
            {
                const int rowCount = 20;
                for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
                {
                    int columnCount = rowCount - rowIndex;
                    for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
                    {
                        var bodyDescription = new BodyDescription
                        {
                            LocalInertia = boxInertia,
                            Pose = new RigidPose
                            {
                                Position = new Vector3(
                                    (-columnCount * 0.5f + columnIndex) * boxShape.Width,
                                    (rowIndex + 0.5f) * boxShape.Height,
                                    (pyramidIndex - pyramidCount * 0.5f) * (boxShape.Length + 4)),
                                Orientation = BepuUtilities.Quaternion.Identity
                            },
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0 }, //Note no sleeping.
                            Collidable = new CollidableDescription { Shape = boxIndex, SpeculativeMargin = .1f }
                        };
                        Simulation.Bodies.Add(ref bodyDescription);
                    }
                }
            }

            var staticShape = new Box(200, 1, 200);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);

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
            Simulation.Statics.Add(ref staticDescription);
        }

    }
}
