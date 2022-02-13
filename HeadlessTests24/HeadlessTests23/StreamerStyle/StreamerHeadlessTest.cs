using HeadlessTests23.StreamerStyle.Actions;
using System.Diagnostics;

namespace HeadlessTests23.StreamerStyle;
static class StreamerHeadlessTest
{
    public static void Test<TScene, TAction>(int randomSeed, int threadCount, int runCount, int framesBeforeMeasurement, int framesToMeasure, List<double> times) where TScene : Scene, new() where TAction : IAction, new()
    {
        var runFrameTimes = new double[runCount];
        for (int runIndex = -1; runIndex < runCount; ++runIndex)
        {
            var scene = new TScene();
            var random = new Random(randomSeed);
            scene.Initialize(random, threadCount);
            var action = new TAction();
            action.Initialize(random, scene);
            Console.Write($"Preframes for {runIndex} completed: ");
            GC.Collect(3, GCCollectionMode.Forced, true, true);
            float accumulatedTime = 0;
            for (int i = 0; i < framesBeforeMeasurement; ++i)
            {
                scene.Update();
                accumulatedTime += scene.TimestepDuration;
                action.Update(scene, random, accumulatedTime);
                if (i % 32 == 0)
                    Console.Write($"{i}, ");
            }
            Console.WriteLine($"Preframes for {runIndex} complete");
            double time = 0;
            Console.Write("Completed frames: ");
            for (int i = 0; i < framesToMeasure; ++i)
            {
                var start = Stopwatch.GetTimestamp();
                scene.Update();
                var end = Stopwatch.GetTimestamp();
                accumulatedTime += scene.TimestepDuration;
                action.Update(scene, random, accumulatedTime);
                time += (end - start) / (double)Stopwatch.Frequency;
                if (i % 32 == 0)
                    Console.Write($"{i}, ");
            }
            Console.WriteLine();
            var frameTime = time / framesToMeasure;
            Console.WriteLine($"Time per frame average (ms): {1e3 * frameTime}");
            if (runIndex >= 0)
                runFrameTimes[runIndex] = frameTime;
            scene.Dispose();
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