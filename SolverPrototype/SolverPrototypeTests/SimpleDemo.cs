using DemoRenderer;
using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace SolverPrototypeTests
{
    public class SimpleDemo : Demo
    {
        public override void Initialize(Camera camera)
        {
            Simulation = new Simulation(BufferPool);
            const int width = 32;
            const int height = 32;
            const int length = 32;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(3), new Vector3(), 1),
                new BallSocketConstraintBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            //Simulation.Solver.IterationCount = 8;
            camera.Position = new Vector3(-40, -10, 5);
            camera.Yaw += MathF.PI * 0.65f;
            camera.Pitch += MathF.PI * -0.2f;

        }
        
    }
}
