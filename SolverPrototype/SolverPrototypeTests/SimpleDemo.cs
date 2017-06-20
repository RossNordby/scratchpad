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
            const int width = 4;
            const int height = 4;
            const int length = 4;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(3), new Vector3()),
                new BallSocketConstraintBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            camera.Position = new Vector3(0, 0, 10);
        }
    
    }
}
