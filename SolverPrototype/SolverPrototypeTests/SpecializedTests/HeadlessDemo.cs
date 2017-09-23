using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace SolverPrototypeTests.SpecializedTests
{
    static class HeadlessDemo
    {
        public static void Simple()
        {
            var simpleDemo = new SimpleDemo();
            simpleDemo.Initialize(new DemoRenderer.Camera(1, 1, 1, 1));
            const int frameCount = 1000;
            simpleDemo.Update(1 / 60f);
            simpleDemo.Update(1 / 60f);
            double time = 0;
            for (int i = 0; i < frameCount; ++i)
            {
                CacheBlaster.Blast();
                var start = Stopwatch.GetTimestamp();
                simpleDemo.Update(1 / 60f);
                var end = Stopwatch.GetTimestamp();
                time += (end - start) / (double)Stopwatch.Frequency;
            }
            Console.WriteLine($"Time per frame (us): {1e6 * time / frameCount}");
        }
    }
}
