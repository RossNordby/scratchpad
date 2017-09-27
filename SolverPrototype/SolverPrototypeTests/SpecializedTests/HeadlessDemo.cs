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
            //for (int i = 0; i < 128; ++i)
            //{
            //    simpleDemo.Update(1 / 60f);
            //}
            double time = 0;
            const int frameCount = 1000;
            for (int i = 0; i < frameCount; ++i)
            {
                //CacheBlaster.Blast();
                var start = Stopwatch.GetTimestamp();
                simpleDemo.Update(1 / 60f);
                var end = Stopwatch.GetTimestamp();
                time += (end - start) / (double)Stopwatch.Frequency;
                //Console.WriteLine($"FRAME {i}, time (us): {1e6 * simpleDemo.Simulation.Timings[simpleDemo.Simulation.NarrowPhase]}");
                Console.WriteLine($"FRAME {i}");
            }
            Console.WriteLine($"Time per frame (us): {1e6 * time / frameCount}");
            simpleDemo.Dispose();
        }
    }
}
