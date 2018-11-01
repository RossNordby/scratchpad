using System;
using System.Diagnostics;

namespace GoingWide
{
    class Program
    {
        static void Run<T>(int runCount, int iterationCount) where T : Benchmark, new()
        {
            var name = typeof(T).Name;
            Console.WriteLine($"Testing {name}...");
            var benchmark = new T();

            //Do a bit of warmup. Both to JIT and to give the processor time to adjust clocks for the instruction stream.
            for (int i = 0; i < iterationCount; ++i)
            {
                benchmark.Execute();
            }
            benchmark.Reset();
            var minimumTime = double.MaxValue;
            var maximumTime = double.MinValue;
            for (int j = 0; j < runCount; ++j)
            {
                var start = Stopwatch.GetTimestamp();
                for (int i = 0; i < iterationCount; ++i)
                {
                    benchmark.Execute();
                }
                var end = Stopwatch.GetTimestamp();
                benchmark.Reset();
                var time = (end - start) / (double)Stopwatch.Frequency;

                Console.WriteLine($"Time per iteration (ms): {1e3 * time / iterationCount}");
                if (minimumTime > time)
                    minimumTime = time;
                if (maximumTime < time)
                    maximumTime = time;

            }
            Console.WriteLine($"{name} time per iteration min (ms): {1e3 * minimumTime / iterationCount}, max (ms) {1e3 * maximumTime / iterationCount}");
        }
        static void Main(string[] args)
        {
            const int runCount = 5;
            const int iterationCount = 10;
            Run<SOA>(runCount, iterationCount);
            Run<AOSScalar>(runCount, iterationCount);
            Run<AOSSSE>(runCount, iterationCount);
            Run<AOSNumerics>(runCount, iterationCount);
            Run<AOSOAU>(runCount, iterationCount);
            Run<AOSOALS>(runCount, iterationCount);
            Run<AOSOANumerics>(runCount, iterationCount);
        }
    }
}
