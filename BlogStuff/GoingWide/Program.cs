﻿using System;
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

            benchmark.Execute();
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
            Run<AOSScalar>(runCount, iterationCount);
            Run<AOSSSE>(runCount, iterationCount);
        }
    }
}
