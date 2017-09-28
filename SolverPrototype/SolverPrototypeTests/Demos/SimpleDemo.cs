using BEPUutilities2;
using DemoRenderer;
using DemoUtilities;
using SolverPrototype;
using SolverPrototype.Collidables;
using System.Numerics;

namespace SolverPrototypeTests
{
    public class SimpleDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-6, 10, -6);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            var shape = new Sphere(0.5f);
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 32;
            const int height = 32;
            const int length = 32;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1.2f, 1.2f, 1.2f), new Vector3(1, 1, 1), 1f, shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            //SimulationScrambling.AddRemoveChurn<BallSocket>(Simulation, 100, bodyHandles, constraintHandles);
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
            //velocity.Linear = new Vector3(.1f, 0, 0.1f);
            //velocity.Angular = new Vector3();
            //Simulation.Bodies.SetVelocity(bodyHandles[width], ref velocity);
            //Simulation.Solver.IterationCount = 100;
            //camera.Position = new Vector3(-40, -10, 5);
            //camera.Yaw += MathF.PI * 0.65f;
            //camera.Pitch += MathF.PI * -0.2f;

        }


        public override void Update(Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.P))
            {
                unsafe { var accessViolationSuppressant = stackalloc int[0]; }
                BodyVelocity velocity;
                velocity.Linear = new Vector3(.1f, 0, 0.1f);
                velocity.Angular = new Vector3();
                Simulation.Bodies.SetVelocity(64, ref velocity);
            }
            base.Update(input, dt);

        }

    }
}
