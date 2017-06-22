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
        static float GetDistance()
        {
            var lineStart = new Vector3(2, 3, 1);
            var lineDirection = new Vector3(-1, 0, 1);
            float lineLength = 4;

            var rayDirection = new Vector3(0, 0, 1.5f);

            //Treat the view ray as a plane. Construct the plane's normal from the rayDirection and vector of closest approach between the two lines (lineDirection x rayDirection).
            //The plane normal is:
            //N = rayDirection x (lineDirection x rayDirection) / ||lineDirection x rayDirection||
            //(The vector triple product has some identities, but bleh.)
            //tLine = dot(lineStart - origin, N) / -dot(N, lineDirection)
            //Doing some algebra and noting that the origin is 0 here, that becomes:
            //tLine = dot(lineStart, rayDirection x (lineDirection x rayDirection)) / -||lineDirection x rayDirection||^2

            var lineCrossRay = Vector3.Cross(lineDirection, rayDirection);
            var numer = Vector3.Cross(rayDirection, lineCrossRay);
            var denom = -lineCrossRay.LengthSquared();
            //If the lines are parallel, just use the line start.
            float tLine = denom > -1e-7f ? 0 : Vector3.Dot(lineStart, numer) / denom;

            //The true tLine must be from 0 to lineLength.
            tLine = Math.Clamp(tLine, 0, lineLength);

            var closestOnLine = lineStart + tLine * lineDirection;

            var closestOnRay = rayDirection * (Vector3.Dot(closestOnLine, rayDirection) / rayDirection.LengthSquared());
            return Vector3.Distance(closestOnRay, closestOnLine);
        }

        public override void Initialize(Camera camera)
        {
            var distance = GetDistance();
            Simulation = new Simulation(BufferPool);
            const int width = 32;
            const int height = 32;
            const int length = 32;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(3), new Vector3()),
                new BallSocketConstraintBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            //Simulation.Solver.IterationCount = 8;
            camera.Position = new Vector3(0, 0, 10);

        }
        
    }
}
