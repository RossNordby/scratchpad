using BEPUutilities2;
using SolverPrototype;
using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;

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
        public static TestTimings Solve(int width, int height, int length, int frameCount, int threadCount, IThreadDispatcher initializationThreadPool, IThreadDispatcher threadPool)
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);
            GC.Collect(3, GCCollectionMode.Forced, true);
            SimulationSetup.BuildLattice(width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);

            SimulationSetup.ScrambleBodies(simulation);
            SimulationSetup.ScrambleConstraints(simulation.Solver);
            SimulationSetup.ScrambleBodyConstraintLists(simulation);
            SimulationSetup.AddRemoveChurn(simulation, 100000, bodyHandles, constraintHandles);


            //Attempt cache optimization.
            int bodyOptimizationIterations = bodyHandles.Length * 1;
            //simulation.BodyLayoutOptimizer.DumbIncrementalOptimize(); //prejit
            //var timer = Stopwatch.StartNew();
            //simulation.BodyLayoutOptimizer.PartialIslandOptimizeDFS(simulation.Bodies.BodyCount);
            for (int i = 0; i < bodyOptimizationIterations; ++i)
            {
                //simulation.BodyLayoutOptimizer.DumbIncrementalOptimize();
                //simulation.BodyLayoutOptimizer.SortingIncrementalOptimize(simulation.BufferPool);
                //simulation.BodyLayoutOptimizer.PartialIslandOptimizeDFS();
                simulation.BodyLayoutOptimizer.IncrementalOptimize(32, initializationThreadPool, simulation.BufferPool);
            }
            //int bodyOptimizationIterations = 32;
            //for (int i = 0; i < bodyOptimizationIterations; ++i)
            //{
            //    simulation.BodyLayoutOptimizer.DumbOptimizeMultithreaded(bodyHandles.Length, threadPool, simulation.BufferPool);
            //}
            //timer.Stop();
            //var optimizationTime = timer.Elapsed.TotalSeconds;
            //Console.WriteLine($"Finished {bodyOptimizationIterations} body optimizations, time (ms): {optimizationTime * 1e3}, per iteration (us): {optimizationTime * 1e6 / bodyOptimizationIterations}");

            int constraintCount = 0;
            for (int i = 0; i < simulation.Solver.Batches.Count; ++i)
            {
                for (int j = 0; j < simulation.Solver.Batches[i].TypeBatches.Count; ++j)
                {
                    constraintCount += simulation.Solver.Batches[i].TypeBatches[j].ConstraintCount;
                }
            }
            const int bundlesPerOptimizationRegion = 256;
            int constraintsPerOptimizationRegion = bundlesPerOptimizationRegion * Vector<int>.Count;
            const int regionsPerConstraintOptimizationIteration = 8;
            int constraintOptimizationIterations = 361;
            //int constraintOptimizationIterations = Math.Max(16,
            //    (int)(1 * 2 * ((long)constraintCount * constraintCount /
            //    ((double)constraintsPerOptimizationRegion * constraintsPerOptimizationRegion)) / regionsPerConstraintOptimizationIteration));

            //simulation.ConstraintLayoutOptimizer.Update(2, 1, simulation.BufferPool); //prejit
            //var constraintsToOptimize = constraintsPerOptimizationRegion * regionsPerConstraintOptimizationIteration * constraintOptimizationIterations;
            //timer.Restart();
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                simulation.ConstraintLayoutOptimizer.Update(bundlesPerOptimizationRegion, regionsPerConstraintOptimizationIteration, simulation.BufferPool, initializationThreadPool);
            }
            //timer.Stop();
            //Console.WriteLine($"Finished constraint optimizations, time (ms): {timer.Elapsed.TotalMilliseconds}" +
            //    $", per iteration (us): {timer.Elapsed.TotalSeconds * 1e6 / constraintOptimizationIterations}");

            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            simulation.Solver.IterationCount = iterationCount;
            
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
        static void Subtest(int width, int height, int length, int frameCount, IThreadDispatcher initializationThreadPool, StreamWriter writer)
        {
            const int testsPerVariant = 8;
            WriteLine(writer, $"{width}x{height}x{length} lattice, {frameCount} frames:");
            var timings = new TestTimings[Environment.ProcessorCount];


            //for (int threadCount = 1; threadCount <= 1; ++threadCount)
            for (int threadCount = 1; threadCount <= Environment.ProcessorCount; ++threadCount)
            {
                var threadPool = new SimpleThreadDispatcher(threadCount);
                ref var timingsForThreadCount = ref timings[threadCount - 1];
                timingsForThreadCount = new TestTimings { Total = double.MaxValue };
                for (int i = 0; i < testsPerVariant; ++i)
                {
                    var candidateTimings = Solve(width, height, length, frameCount, threadCount, initializationThreadPool, threadPool);
                    WriteLine(writer, $"{i} AVE: {Math.Round(1e3 * candidateTimings.Average, 2)}, MIN: {Math.Round(1e3 * candidateTimings.Min, 2)}, MAX: {Math.Round(1e3 * candidateTimings.Max, 2)}, STD DEV: {Math.Round(1e3 * candidateTimings.StdDev, 3)}, ");
                    if (candidateTimings.Total < timingsForThreadCount.Total)
                        timingsForThreadCount = candidateTimings;
                }
                WriteLine(writer, $"{threadCount}T: {Math.Round(timingsForThreadCount.Total * 1e3, 2)}");
                threadPool.Dispose();
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
            var initializationThreadPool = new SimpleThreadDispatcher(Environment.ProcessorCount);
            Subtest(32, 32, 32, 8, initializationThreadPool, writer);
            Subtest(26, 26, 26, 12, initializationThreadPool, writer);
            Subtest(20, 20, 20, 20, initializationThreadPool, writer);
            Subtest(16, 16, 16, 30, initializationThreadPool, writer);
            Subtest(13, 13, 13, 45, initializationThreadPool, writer);
            Subtest(10, 10, 10, 70, initializationThreadPool, writer);
            initializationThreadPool.Dispose();
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
