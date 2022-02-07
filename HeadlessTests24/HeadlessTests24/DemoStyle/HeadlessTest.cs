using System.Diagnostics;

namespace HeadlessTests24.DemoStyle;
static class HeadlessTest
{
    public static void Test<T>(int threadCount, int runCount, int framesBeforeMeasurement, int framesToMeasure, List<double> times) where T : Demo, new()
    {
        var runFrameTimes = new double[runCount];
        for (int runIndex = -1; runIndex < runCount; ++runIndex)
        {
            var demo = new T();
            demo.Initialize(threadCount);
            Console.Write($"Preframes for {runIndex} completed: ");
            GC.Collect(3, GCCollectionMode.Forced, true, true);
            for (int i = 0; i < framesBeforeMeasurement; ++i)
            {
                demo.Update();
                if (i % 32 == 0)
                    Console.Write($"{i}, ");
            }
            Console.WriteLine($"Preframes for {runIndex} complete");
            double time = 0;
            Console.Write("Completed frames: ");
            for (int i = 0; i < framesToMeasure; ++i)
            {
                var start = Stopwatch.GetTimestamp();
                demo.Update();
                var end = Stopwatch.GetTimestamp();
                time += (end - start) / (double)Stopwatch.Frequency;
                if (i % 32 == 0)
                    Console.Write($"{i}, ");
            }
            Console.WriteLine();
            var frameTime = time / framesToMeasure;
            Console.WriteLine($"Time per frame average (ms): {1e3 * frameTime}");
            if (runIndex >= 0)
                runFrameTimes[runIndex] = frameTime;
            demo.Dispose();
        }
        var min = double.MaxValue;
        var max = double.MinValue;
        var sum = 0.0;
        var sumOfSquares = 0.0;
        for (int runIndex = 0; runIndex < runCount; ++runIndex)
        {
            var time = runFrameTimes[runIndex];
            min = Math.Min(time, min);
            max = Math.Max(time, max);
            sum += time;
            sumOfSquares += time * time;
        }
        var average = sum / runCount;
        var stdDev = Math.Sqrt(sumOfSquares / runCount - average * average);
        Console.WriteLine($"Average (ms): {average * 1e3}");
        Console.WriteLine($"Min, max (ms): {min * 1e3}, {max * 1e3}");
        Console.WriteLine($"Std Dev (ms): {stdDev * 1e3}");
        times.AddRange(runFrameTimes);
    }
}