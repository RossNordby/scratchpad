﻿using System;
using System.Diagnostics;
using System.IO;
using System.Linq;

namespace SolverPrototypeTests
{
    public static class AutoTester
    {
        public struct TestTimings
        {
            public double Total;
            public double Min;
            public double Max;
            public double Average;
            public double StdDev;
        }
        public static TestTimings Solve(int width, int height, int length, int frameCount, int threadCount)
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);
            GC.Collect(3, GCCollectionMode.Forced, true);
            SimulationSetup.BuildLattice(width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);


            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            simulation.Solver.IterationCount = iterationCount;


            //var threadPool = new TPLPool(8);
            var threadPool = new SimpleThreadPool(threadCount);
            //var threadPool = new NotQuiteAThreadPool();
            double totalTime = 0;
            double sumOfSquares = 0.0;
            TestTimings testTimings;
            testTimings.Min = double.MaxValue;
            testTimings.Max = double.MinValue;
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                CacheBlaster.Blast();
                var frameStartTime = Stopwatch.GetTimestamp();
                //simulation.Solver.Update(dt, inverseDt);
                //simulation.Solver.NaiveMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.ManualNaiveMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.IntermediateMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.NaiveMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.MultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.ContiguousClaimMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                simulation.Solver.MultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                var frameEndTime = Stopwatch.GetTimestamp();
                var frameTime = (frameEndTime - frameStartTime) / (double)Stopwatch.Frequency;
                if (frameTime < testTimings.Min)
                    testTimings.Min = frameTime;
                if (frameTime > testTimings.Max)
                    testTimings.Max = frameTime;
                //Console.WriteLine($"F{frameIndex}: {1e3 * frameTime}");
                totalTime += frameTime;
                sumOfSquares += frameTime * frameTime;
            }

            threadPool.Dispose();
            simulation.BufferPool.Clear();

            testTimings.Average = totalTime / frameCount;
            testTimings.Total = totalTime;
            testTimings.StdDev = Math.Sqrt(sumOfSquares / frameCount - testTimings.Average * testTimings.Average);

            return testTimings;
        }

        static void WriteLine(StreamWriter writer, string text)
        {
            writer.WriteLine(text);
            Console.WriteLine(text);
        }
        static void Subtest(int width, int height, int length, int frameCount, StreamWriter writer)
        {
            const int testsPerVariant = 3;
            WriteLine(writer, $"{width}x{height}x{length} lattice, {frameCount} frames:");
            var timings = new TestTimings[Environment.ProcessorCount];
            //for (int threadCount = 1; threadCount <= 1; ++threadCount)
            for (int threadCount = 1; threadCount <= Environment.ProcessorCount; ++threadCount)
            {
                ref var timingsForThreadCount = ref timings[threadCount - 1];
                timingsForThreadCount = new TestTimings { Total = double.MaxValue };
                for (int i = 0; i < testsPerVariant; ++i)
                {
                    var candidateTimings = Solve(width, height, length, frameCount, threadCount);
                    //WriteLine(writer, $"{i} AVE: {Math.Round(1e3 * candidateTimings.Average, 2)}, MIN: {Math.Round(1e3 * candidateTimings.Min, 2)}, MAX: {Math.Round(1e3 * candidateTimings.Max, 2)}, STD DEV: {Math.Round(1e3 * candidateTimings.StdDev, 3)}, ");
                    if (candidateTimings.Total < timingsForThreadCount.Total)
                        timingsForThreadCount = candidateTimings;
                }
                WriteLine(writer, $"{threadCount}T: {Math.Round(timingsForThreadCount.Total * 1e3, 2)}");
            }
            int fastestIndex = 0;
            for (int i = 1; i < timings.Length; ++i)
            {
                if (timings[i].Total < timings[fastestIndex].Total)
                    fastestIndex = i;
            }
            WriteLine(writer, $"Scaling: {timings[0].Total / timings[fastestIndex].Total}");
        }

        public static void Test()
        {
            var memoryStream = new MemoryStream();
            var writer = new StreamWriter(memoryStream);
            //Subtest(32, 32, 32, 8, writer);
            //Subtest(26, 26, 26, 12, writer);
            //Subtest(20, 20, 20, 20, writer);
            //Subtest(16, 16, 16, 30, writer);
            //Subtest(13, 13, 13, 45, writer);
            Subtest(10, 10, 10, 70, writer);
            writer.Flush();
            var path = "log.txt";
            using (var stream = File.OpenWrite(path))
            {
                Console.WriteLine($"Writing results to path: {Path.GetFullPath(path)}");
                var bytes = memoryStream.ToArray();
                stream.Write(bytes, 0, bytes.Length);
            }
            Console.ReadKey();
        }
    }
}
