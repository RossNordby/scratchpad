using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    public static class AutoTester
    {
        public struct TestTimings
        {
            public double SetupAndSolveTime;
            public double SolveTime;
        }
        public static TestTimings Solve(int width, int height, int length , int frameCount, int threadCount)
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);

            SimulationSetup.BuildLattice(width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);


            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            simulation.Solver.IterationCount = iterationCount;


            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            var timer = new Stopwatch();
            //var threadPool = new TPLPool(8);
            var threadPool = new SimpleThreadPool(threadCount);
            //var threadPool = new NotQuiteAThreadPool();
            double solveTime = 0;
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                //CacheBlaster.Blast();
                timer.Start();
                //simulation.Solver.Update(dt, inverseDt);
                //solveTime += simulation.Solver.ManualNaiveMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //solveTime += simulation.Solver.IntermediateMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.NaiveMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //solveTime += simulation.Solver.MultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //solveTime += simulation.Solver.ContiguousClaimMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                solveTime += simulation.Solver.LastMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                timer.Stop();
            }

            threadPool.Dispose();
            simulation.BufferPool.Clear();

            return new TestTimings { SetupAndSolveTime = timer.Elapsed.TotalSeconds, SolveTime = solveTime };
        }

        static void WriteLine(StreamWriter writer, string text)
        {
            writer.WriteLine(text);
            Console.WriteLine(text);
        }
        static void Subtest(int width, int height, int length, int frameCount, StreamWriter writer)
        {
            const int testsPerVariant = 8;
            WriteLine(writer, $"{width}x{height}x{length} lattice, {frameCount} frames:");
            var timings = new TestTimings[Environment.ProcessorCount];
            for (int threadCount = 1; threadCount <= Environment.ProcessorCount; ++threadCount)
            {
                ref var timingsForThreadCount = ref timings[threadCount - 1];
                timingsForThreadCount = new TestTimings { SetupAndSolveTime = double.MaxValue, SolveTime = double.MaxValue };
                for (int i = 0; i < testsPerVariant; ++i)
                {
                    var candidateTimings = Solve(width, height, length, frameCount, threadCount);
                    if (candidateTimings.SetupAndSolveTime < timingsForThreadCount.SetupAndSolveTime)
                        timingsForThreadCount = candidateTimings;
                }
                WriteLine(writer, $"{threadCount}T: {Math.Round(timingsForThreadCount.SetupAndSolveTime * 1e3, 2)}, {Math.Round((timingsForThreadCount.SetupAndSolveTime - timingsForThreadCount.SolveTime) * 1e3, 2)}");
            }
            int slowestIndex = 0;
            int fastestIndex = 0;
            for (int i = 1; i < timings.Length; ++i)
            {
                if (timings[i].SolveTime < timings[fastestIndex].SetupAndSolveTime)
                    fastestIndex = i;
                if (timings[i].SolveTime > timings[slowestIndex].SetupAndSolveTime)
                    slowestIndex = i;
            }
            WriteLine(writer, $"Scaling: {timings[slowestIndex].SetupAndSolveTime / timings[fastestIndex].SetupAndSolveTime}");
        }

        public static void Test()
        {
            var memoryStream = new MemoryStream();
            var writer = new StreamWriter(memoryStream);
            WriteLine(writer, "N threads: total solve+setup time in ms, setup time in ms");
            Subtest(32, 32, 32, 8, writer);
            Subtest(26, 26, 26, 12, writer);
            Subtest(20, 20, 20, 20, writer);
            Subtest(16, 16, 16, 30, writer);
            Subtest(13, 13, 13, 45, writer);
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
