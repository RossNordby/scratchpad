using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace SolverPrototypeTests
{
    public class SimpleDemo : Demo
    {
        public override void Initialize()
        {
            Simulation = new Simulation(BufferPool);
            const int width = 4;
            const int height = 4;
            const int length = 4;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3()),
                new BallSocketConstraintBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
        }
    
    }
}
