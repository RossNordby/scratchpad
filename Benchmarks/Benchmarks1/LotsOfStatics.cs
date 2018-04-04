using BenchmarkDotNet.Attributes.Exporters;
using BenchmarkDotNet.Attributes.Jobs;
using BenchmarkDotNet.Engines;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using BEPUutilities;

namespace Benchmarks1
{
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 8, invocationCount: 128, id: nameof(LotsOfStatics) + " v1")]
    public class LotsOfStatics : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Space.ForceUpdater.Gravity = new Vector3(0, -10, 0);
            Space.Solver.IterationLimit = 8;
            var shape = new BoxShape(1, 1, 1);
            const int width = 64;
            const int height = 1;
            const int length = 64;
            Space.DeactivationManager.MaximumDeactivationAttemptsPerFrame = width * height * length;
            Space.DeactivationManager.LowVelocityTimeMinimum = 0.1f;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var entity = new Entity(shape, 1);
                        entity.Position = new Vector3(1.4f, 1.1f, 1.4f) * new Vector3(i, j, k) + new Vector3(-width * 0.7f, 2.5f, -length * 0.7f);
                        Space.Add(entity);

                    }
                }
            }
            var staticShape = new BoxShape(1, 1, 1);
            const int staticGridWidth = 96;
            const float staticSpacing = 1.2f;
            var gridOffset = -0.5f * staticGridWidth * staticSpacing;
            for (int i = 0; i < staticGridWidth; ++i)
            {
                for (int j = 0; j < staticGridWidth; ++j)
                {
                    var entity = new Entity(shape);
                    entity.Position = new Vector3(
                                gridOffset + i * staticSpacing,
                                -0.707f,
                                gridOffset + j * staticSpacing);
                    entity.Orientation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1 + i, i * j % 10, -10 + -j)), (i ^ j) * 0.5f * (MathHelper.PiOver4));
                    Space.Add(entity);
                }
            }
            for (int i = 0; i < 512; ++i)
            {
                //We want the benchmark to start after things have had a chance to go to sleep naturally.
                Space.Update();
            }          
        }
    }
}
