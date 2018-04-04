using BenchmarkDotNet.Attributes.Jobs;
using BenchmarkDotNet.Engines;
using BEPUphysics.CollisionShapes.ConvexShapes;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;

namespace Benchmarks1
{
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 4, invocationCount: 128, id: nameof(ShapePile) + " v1")]
    public class ShapePile : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Space.ForceUpdater.Gravity = new Vector3(0, -10, 0);
            Space.Solver.IterationLimit = 8;
            var box = new BoxShape(1f, 3f, 2f);
            var capsule = new CapsuleShape(1f, 1f);
            var sphere = new SphereShape(1.5f);
            const int width = 16;
            const int height = 16;
            const int length = 16;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var position = new Vector3(3, 3, 3) * new Vector3(i, j, k) + new Vector3(-width * 1.5f, 1.5f, -length * 1.5f);
                        switch (j % 3)
                        {
                            case 0:
                                Space.Add(new Entity(box, 1) { Position = position });
                                break;
                            case 1:
                                Space.Add(new Entity(capsule, 1) { Position = position });
                                break;
                            case 2:
                                Space.Add(new Entity(sphere, 1) { Position = position });
                                break;
                        }
                    }
                }
            }

            Space.Add(new Box(new Vector3(0, -0.5f, 0), 200, 1, 200));

            //Primarily interested in the more complex simulation that occurs after everything has collapsed.
            for (int i = 0; i < 256; ++i)
            {
                Space.Update();
            }
        }
    }
}
