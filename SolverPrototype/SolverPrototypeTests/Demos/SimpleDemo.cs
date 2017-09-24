using DemoRenderer;
using SolverPrototype;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using SolverPrototypeTests.SpecializedTests;
using System;
using System.Diagnostics;
using System.Numerics;

namespace SolverPrototypeTests
{
    public class SimpleDemo : Demo
    {
        //struct RemovedConstraint<TDescription>
        //{
        //    public TDescription Description;
        //    public int A;
        //    public int B;
        //}

        public unsafe override void Initialize(Camera camera)
        {
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            var shape = new Sphere(0.5f);
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 4;
            const int height = 4;
            const int length = 4;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1.2f, 2, 1.2f), new Vector3(), 1f, shapeIndex),
                new BallSocketConstraintBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            SimulationScrambling.AddRemoveChurn<BallSocket>(Simulation, 100, bodyHandles, constraintHandles);
            //var removeCount = constraintHandles.Length / 4;
            //var removedConstraints = new RemovedConstraint<BallSocket>[removeCount];
            //var removeStart = Stopwatch.GetTimestamp();
            //var handles = stackalloc int[2];
            //for (int i = 0; i < removeCount; ++i)
            //{
            //    Simulation.Solver.GetConstraintReference(constraintHandles[i], out var reference);
            //    var bodyHandleCollector = new ConstraintBodyHandleCollector(Simulation.Bodies, handles);
            //    reference.TypeBatch.EnumerateConnectedBodyIndices(reference.IndexInTypeBatch, ref bodyHandleCollector);
            //    removedConstraints[i].A = bodyHandleCollector.Handles[0];
            //    removedConstraints[i].B = bodyHandleCollector.Handles[1];
            //    Simulation.Solver.GetDescription(constraintHandles[i], out removedConstraints[i].Description);
            //    Simulation.RemoveConstraint(constraintHandles[i]);
            //}
            //var addStart = Stopwatch.GetTimestamp();
            //for (int i = 0; i < removeCount; ++i)
            //{
            //    constraintHandles[i] = Simulation.Add(removedConstraints[i].A, removedConstraints[i].B, ref removedConstraints[i].Description);
            //}

            //var addEnd = Stopwatch.GetTimestamp();
            //Console.WriteLine($"Remove time (us): {1e6 * (addStart - removeStart) / (Stopwatch.Frequency * removeCount)}");
            //Console.WriteLine($"Add time (us): {1e6 * (addEnd - addStart) / (Stopwatch.Frequency * removeCount)}");
            //BodyVelocity velocity;
            //velocity.Linear = new Vector3(10.1f, 0, 0);
            //velocity.Angular = new Vector3();
            //Simulation.Bodies.SetVelocity(bodyHandles[2], ref velocity);
            Simulation.Solver.IterationCount = 6;
            //camera.Position = new Vector3(-40, -10, 5);
            //camera.Yaw += MathF.PI * 0.65f;
            //camera.Pitch += MathF.PI * -0.2f;

        }

        //double time;
        //public override void Update(float dt)
        //{
        //    BodyVelocity velocity;
        //    time += dt;
        //    velocity.Linear = new Vector3((float)Math.Sin(time), 0, 0);
        //    velocity.Angular = new Vector3();
        //    Simulation.Bodies.SetVelocity(0, ref velocity);
        //    base.Update(dt);

        //}

    }
}
