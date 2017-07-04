using DemoRenderer;
using SolverPrototype;
using SolverPrototype.Collidables;
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
            var shape = new Sphere(0.5f);
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 12;
            const int height = 12;
            const int length = 12;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(3), new Vector3(), 1f, shapeIndex),
                new BallSocketConstraintBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            //BodyVelocity velocity;
            //velocity.Linear = new Vector3(1, 0, 0);
            //velocity.Angular = new Vector3();
            //Simulation.Bodies.SetVelocity(1, ref velocity);
            Simulation.Solver.IterationCount = 8;
            //camera.Position = new Vector3(-40, -10, 5);
            //camera.Yaw += MathF.PI * 0.65f;
            //camera.Pitch += MathF.PI * -0.2f;

        }
        
    }
}
