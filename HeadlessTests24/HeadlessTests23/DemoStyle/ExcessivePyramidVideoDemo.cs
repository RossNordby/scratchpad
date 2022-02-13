using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using System.Numerics;

namespace HeadlessTests23.DemoStyle;

/// <summary>
/// A pyramid of boxes, because you can't have a physics engine without pyramids of boxes.
/// </summary>
public class ExcessivePyramidVideoDemo : Demo
{
    public unsafe override void Initialize(int threadCount)
    {
        ThreadDispatcher = new ThreadDispatcher(threadCount);
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1), frictionCoefficient: 2), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper(), 8);

        var boxShape = new Box(1, 1, 1);
        boxShape.ComputeInertia(1, out var boxInertia);
        var boxIndex = Simulation.Shapes.Add(boxShape);
        const int pyramidCount = 420;
        for (int pyramidIndex = 0; pyramidIndex < pyramidCount; ++pyramidIndex)
        {
            const int rowCount = 20;
            for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
            {
                int columnCount = rowCount - rowIndex;
                for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(
                        (-columnCount * 0.5f + columnIndex) * boxShape.Width,
                        (rowIndex + 0.5f) * boxShape.Height,
                        (pyramidIndex - pyramidCount * 0.5f) * (boxShape.Length + 4)),
                        boxInertia, new CollidableDescription(boxIndex, 0.1f), new(0.01f)));
                }
            }
        }
        Console.WriteLine($"bodies count: {Simulation.Bodies.ActiveSet.Count}");

        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new (Simulation.Shapes.Add(new Box(2500, 1, 2500)), 0.1f)));
    }

    int frameCount;
    public override void Update()
    {
        ++frameCount;
        if (frameCount == 128)
        {
            var bulletShape = new Sphere(6);
            bulletShape.ComputeInertia(5000000, out var inertia);
            var bodyDescription = BodyDescription.CreateDynamic(
                new RigidPose(new Vector3(0, 8, -1200)), new BodyVelocity(new Vector3(0, 0, 230)), inertia, new(Simulation.Shapes.Add(bulletShape), 0.1f), new BodyActivityDescription(0.01f));
            Simulation.Bodies.Add(bodyDescription);
        }
        base.Update();
    }

}
