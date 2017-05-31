using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Text;

namespace SolverPrototypeTests
{
    public struct TimelineStats
    {
        public double Total;
        public double Average;
        public double Min;
        public double Max;
        public double StdDev;
    }

    public class Timeline
    {
        public readonly double[] Times;

        public Timeline(int frameCount)
        {
            Times = new double[frameCount];
        }

        public TimelineStats ComputeStats()
        {
            TimelineStats stats;
            stats.Total = 0.0;
            var sumOfSquares = 0.0;
            stats.Min = double.MaxValue;
            stats.Max = double.MinValue;
            for (int i = 0; i < Times.Length; ++i)
            {
                var time = Times[i];
                stats.Total += time;
                sumOfSquares += time * time;
                if (time < stats.Min)
                    stats.Min = time;
                if (time > stats.Max)
                    stats.Max = time;
            }
            stats.Average = stats.Total / Times.Length;
            stats.StdDev = Math.Sqrt(Math.Max(0, sumOfSquares / Times.Length - stats.Average * stats.Average));
            return stats;
        }
    }

    public class SimulationTimeSamples
    {
        public Timeline Simulation;
        public Timeline BodyOptimizer;
        public Timeline ConstraintOptimizer;
        public Timeline BatchCompressor;
        public Timeline PoseIntegrator;
        public Timeline Solver;

        public int NextFrame = 0;

        public SimulationTimeSamples(int frameCount)
        {
            Simulation = new Timeline(frameCount);
            BodyOptimizer = new Timeline(frameCount);
            ConstraintOptimizer = new Timeline(frameCount);
            BatchCompressor = new Timeline(frameCount);
            PoseIntegrator = new Timeline(frameCount);
            Solver = new Timeline(frameCount);
        }

        public void RecordFrame(Simulation simulation)
        {
            Simulation.Times[NextFrame] = simulation.Timings[simulation];
            BodyOptimizer.Times[NextFrame] = simulation.Timings[simulation.BodyLayoutOptimizer];
            ConstraintOptimizer.Times[NextFrame] = simulation.Timings[simulation.ConstraintLayoutOptimizer];
            BatchCompressor.Times[NextFrame] = simulation.Timings[simulation.SolverBatchCompressor];
            PoseIntegrator.Times[NextFrame] = simulation.Timings[simulation.PoseIntegrator];
            Solver.Times[NextFrame] = simulation.Timings[simulation.Solver];
            ++NextFrame;
        }
    }
}
