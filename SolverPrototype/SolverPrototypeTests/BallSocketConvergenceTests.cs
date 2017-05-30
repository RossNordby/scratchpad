using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using static SolverPrototype.Solver;
using Quaternion = BEPUutilities2.Quaternion;

namespace SolverPrototypeTests
{
    static class BallSocketConvergenceTests
    {
        public static void Test()
        {
            const int width = 16;
            const int height = 16;
            const int length = 16;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3()),
                new BallSocketConstraintBuilder(),
                width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);
            
            var threadDispatcher = new SimpleThreadDispatcher(8);
            
            var timer = Stopwatch.StartNew();
           
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 32;
            const int frameCount = 2560;
            simulation.Solver.IterationCount = iterationCount;

            simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            timer.Reset();
            
            double solveTime = 0;
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = simulation.Bodies.GetBodyEnergyHeuristic();
                //Update the penetration depths associated with the constraints.
                //This simulates actual position integration and repeated contact detection, allowing the constraints to properly spring.


                timer.Start();
                simulation.Timestep(dt);
                //simulation.Timestep(dt, threadDispatcher);
                timer.Stop();
                var energyAfter = simulation.Bodies.GetBodyEnergyHeuristic();
                int sampledBodyIndex = width;
                simulation.Bodies.GetPose(bodyHandles[sampledBodyIndex], out var samplePose);
                simulation.Bodies.GetVelocity(bodyHandles[sampledBodyIndex], out var sampleVelocity);
                //for (int i =0; i < simulation.Bodies.BodyCount; ++i)
                //{
                //    simulation.Bodies.GetPose(simulation.Bodies.IndexToHandle[i], out var pose);
                //    simulation.Bodies.GetVelocity(simulation.Bodies.IndexToHandle[i], out var velocity);
                //    Console.WriteLine($"Sample {i} position: {pose.Position}, velocity: {velocity.Linear}");
                //}
                Console.WriteLine($"Sample {sampledBodyIndex} position: {samplePose.Position}, velocity: {sampleVelocity.Linear}");

                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");
            }

            Console.WriteLine($"Time (ms): {(1e3 * timer.Elapsed.TotalSeconds)}");
            Console.WriteLine($"Solve time (ms): {1e3 * solveTime}");


            threadDispatcher.Dispose();
            simulation.BufferPool.Clear();

        }



    }
}

