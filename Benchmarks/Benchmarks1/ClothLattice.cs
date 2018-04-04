using BenchmarkDotNet.Attributes.Jobs;
using BenchmarkDotNet.Engines;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;

namespace Benchmarks1
{
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 8, invocationCount: 128)]
    public class ClothLattice : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Space.ForceUpdater.Gravity = new Vector3(0, -10, 0);
            Space.Solver.IterationLimit = 8;
            //Build a grid of shapes to be connected.
            const int width = 128;
            const int length = 128;
            const float spacing = 1.75f;
            Sphere[][] nodes = new Sphere[width][];
            for (int i = 0; i < width; ++i)
            {
                nodes[i] = new Sphere[length];
                for (int j = 0; j < length; ++j)
                {
                    var location = new Vector3(0, 30, 0) + new Vector3(spacing, 0, spacing) * (new Vector3(i, 0, j) + new Vector3(-width * 0.5f, 0, -length * 0.5f));
                    var node = nodes[i][j] = new Sphere(location, 0.5f, 1);
                    node.ActivityInformation.IsAlwaysActive = true;
                    Space.Add(node);
                }
            }
            //Construct some joints between the nodes.
            void AddConstraint(Sphere a, Sphere b)
            {
                var bsj = new BallSocketJoint(a, b, 0.5f * (a.Position + b.Position));
                //Note that I didn't bother computing the exact equivalent spring settings here because I'm lazy. The important thing is to have roughly the same number of collisions.
                bsj.SpringSettings.Damping *= 0.01f;
                bsj.SpringSettings.Stiffness *= 0.01f;
                Space.Solver.Add(bsj);
            }
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < length; ++j)
                {
                    if (i >= 1)
                        AddConstraint(nodes[i][j], nodes[i - 1][j]);
                    if (j >= 1)
                        AddConstraint(nodes[i][j], nodes[i][j - 1]);
                    if (i >= 1 && j >= 1)
                        AddConstraint(nodes[i][j], nodes[i - 1][j - 1]);
                    if (i < width - 1 && j >= 1)
                        AddConstraint(nodes[i][j], nodes[i + 1][j - 1]);
                }
            }
            Space.Add(new Sphere(new Vector3(-10, -15, 0), 25));
            Space.Add(new Box(new Vector3(-0, -10, 0), 200, 1, 200));
        }
    }
}
