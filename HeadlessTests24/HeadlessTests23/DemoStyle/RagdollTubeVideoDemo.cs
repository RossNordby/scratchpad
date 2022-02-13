using BepuUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace HeadlessTests23.DemoStyle
{
    /// <summary>
    /// Subjects a bunch of unfortunate ragdolls to a tumble dry cycle.
    /// </summary>
    public class RagdollTubeVideoDemo : Demo
    {
        public unsafe override void Initialize(int threadCount)
        {
            ThreadDispatcher = new ThreadDispatcher(threadCount);
            var filters = new CollidableProperty<SubgroupCollisionFilter>();
            //Note the lowered material stiffness compared to many of the other demos. Ragdolls aren't made of concrete.
            //Increasing the maximum recovery velocity helps keep deeper contacts strong, stopping objects from interpenetrating.
            //Higher friction helps the bodies clump and flop, rather than just sliding down the slope in the tube.
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks(filters, new PairMaterialProperties { FrictionCoefficient = 2, MaximumRecoveryVelocity = float.MaxValue, SpringSettings = new SpringSettings(10, 1) }), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper(), 4);

            int ragdollIndex = 0;
            var spacing = new Vector3(1.7f, 1.8f, 0.5f);
            int width = 4;
            int height = 4;
            int length = 120;
            var origin = -0.5f * spacing * new Vector3(width - 1, 0, length - 1) + new Vector3(0, 5f, 0);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        RagdollDemo.AddRagdoll(origin + spacing * new Vector3(i, j, k), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.Pi * 0.05f), ragdollIndex++, filters, Simulation);
                    }
                }
            }

            var tubeCenter = new Vector3(0, 8, 0);
            const int panelCount = 20;
            const float tubeRadius = 6;
            var panelShape = new Box(MathF.PI * 2 * tubeRadius / panelCount, 1, 100);
            var panelShapeIndex = Simulation.Shapes.Add(panelShape);
            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, panelCount + 1);
            for (int i = 0; i < panelCount; ++i)
            {
                var rotation = QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, i * MathHelper.TwoPi / panelCount);
                QuaternionEx.TransformUnitY(rotation, out var localUp);
                var position = localUp * tubeRadius;
                builder.AddForKinematic(panelShapeIndex, new (position, rotation), 1);
            }
            builder.AddForKinematic(Simulation.Shapes.Add(new Box(1, 2, panelShape.Length)), new RigidPose(new Vector3(0, tubeRadius - 1, 0)), 0);
            builder.BuildKinematicCompound(out var children);
            var compound = new BigCompound(children, Simulation.Shapes, BufferPool);
            var tubeHandle = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(tubeCenter), new (default, new Vector3(0, 0, .25f)), new (Simulation.Shapes.Add(compound), 0.1f), new BodyActivityDescription(0f)));
            filters[tubeHandle] = new SubgroupCollisionFilter(int.MaxValue);
            builder.Dispose();

            var staticShape = new Box(300, 1, 300);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);
            var staticDescription = new StaticDescription(new Vector3(0, -0.5f, 0), new (staticShapeIndex, 0.1f));
            Simulation.Statics.Add(staticDescription);

            Mesh newtMesh;
            using (var stream = File.Open(@"Content\newt.obj", FileMode.Open))
            {
                newtMesh = MeshLoader.LoadMesh(stream, Vector3.One, BufferPool);
            }
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0.5f, 80), Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI), new (Simulation.Shapes.Add(newtMesh), 0.1f)));

        }

    }
}


