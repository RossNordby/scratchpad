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
    
    /// <summary>
    /// Stores the time it took to complete stages of the physics simulation in a ring buffer. Once the ring buffer is full, the oldest results will be removed.
    /// </summary>
    public class SimulationTimeSamples
    {
        public TimingsRingBuffer Simulation;
        public TimingsRingBuffer BodyOptimizer;
        public TimingsRingBuffer ConstraintOptimizer;
        public TimingsRingBuffer BatchCompressor;
        public TimingsRingBuffer PoseIntegrator;
        public TimingsRingBuffer Solver;
        
        public SimulationTimeSamples(int frameCapacity)
        {
            Simulation = new TimingsRingBuffer(frameCapacity);
            BodyOptimizer = new TimingsRingBuffer(frameCapacity);
            ConstraintOptimizer = new TimingsRingBuffer(frameCapacity);
            BatchCompressor = new TimingsRingBuffer(frameCapacity);
            PoseIntegrator = new TimingsRingBuffer(frameCapacity);
            Solver = new TimingsRingBuffer(frameCapacity);
        }

        public void RecordFrame(DemoSimulation simulation)
        {
            //This requires the simulation to be compiled with profiling enabled.
            Simulation.Add(simulation.Timings[simulation]);
            BodyOptimizer.Add(simulation.Timings[simulation.BodyLayoutOptimizer]);
            ConstraintOptimizer.Add(simulation.Timings[simulation.ConstraintLayoutOptimizer]);
            BatchCompressor.Add(simulation.Timings[simulation.SolverBatchCompressor]);
            PoseIntegrator.Add(simulation.Timings[simulation.PoseIntegrator]);
            Solver.Add(simulation.Timings[simulation.Solver]);
        }
    }
}
