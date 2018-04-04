using BenchmarkDotNet.Attributes.Exporters;
using BenchmarkDotNet.Attributes.Jobs;
using BEPUphysics.Entities.Prefabs;
using BEPUutilities;

namespace Benchmarks1
{
    [SimpleJob(RunStrategy.Monitoring, launchCount: 1, warmupCount: 1, targetCount: 4, invocationCount: 64, id: nameof(Pyramids) + " v1")]
    public class Pyramids : Demo
    {
        public override void IterationSetup()
        {
            CommonSetup();
            Space.ForceUpdater.Gravity = new Vector3(0, -10, 0);
            Space.Solver.IterationLimit = 8;

            const int pyramidCount = 20;
            for (int pyramidIndex = 0; pyramidIndex < pyramidCount; ++pyramidIndex)
            {
                const int rowCount = 20;
                const float boxWidth = 1;
                const float boxHeight = 1;
                const float boxLength = 1;

                for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
                {
                    int columnCount = rowCount - rowIndex;
                    for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
                    {
                        var box = new Box(new Vector3(
                                    (-columnCount * 0.5f + columnIndex) * boxWidth,
                                    (rowIndex + 0.5f) * boxHeight,
                                    (pyramidIndex - pyramidCount * 0.5f) * (boxLength + 4)),
                                    boxWidth, boxHeight, boxLength, 1);
                        //Disable sleeping.
                        box.ActivityInformation.IsAlwaysActive = true;
                        Space.Add(box);
                    }
                }
            }
            Space.Add(new Box(new Vector3(0, -0.5f, 0), 200, 1, 200));            
        }
    }
}
