using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using System.Numerics;

namespace HeadlessTests23.DemoStyle
{
    /// <summary>
    /// A colosseum made out of boxes that is sometimes hit by large purple hail.
    /// </summary>
    public class Colosseum24VideoDemo : Demo
    {
        public static void CreateRingWall(Simulation simulation, Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, int height, float radius)
        {
            var circumference = MathF.PI * 2 * radius;
            var boxCountPerRing = (int)(0.9f * circumference / ringBoxShape.Length);
            float increment = MathHelper.TwoPi / boxCountPerRing;
            for (int ringIndex = 0; ringIndex < height; ringIndex++)
            {
                for (int i = 0; i < boxCountPerRing; i++)
                {
                    var angle = ((ringIndex & 1) == 0 ? i + 0.5f : i) * increment;
                    bodyDescription.Pose = new (position + new Vector3(-MathF.Cos(angle) * radius, (ringIndex + 0.5f) * ringBoxShape.Height, MathF.Sin(angle) * radius), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle));
                    simulation.Bodies.Add(bodyDescription);
                }
            }
        }

        public static void CreateRingPlatform(Simulation simulation, Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, float radius)
        {
            var innerCircumference = MathF.PI * 2 * (radius - ringBoxShape.HalfLength);
            var boxCount = (int)(0.95f * innerCircumference / ringBoxShape.Height);
            float increment = MathHelper.TwoPi / boxCount;
            for (int i = 0; i < boxCount; i++)
            {
                var angle = i * increment;
                bodyDescription.Pose = new (position + new Vector3(-MathF.Cos(angle) * radius, ringBoxShape.HalfWidth, MathF.Sin(angle) * radius),
                    QuaternionEx.Concatenate(QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle + MathF.PI * 0.5f)));
                simulation.Bodies.Add(bodyDescription);
            }
        }

        public static Vector3 CreateRing(Simulation simulation, Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, float radius, int heightPerPlatformLevel, int platformLevels)
        {
            for (int platformIndex = 0; platformIndex < platformLevels; ++platformIndex)
            {
                var wallOffset = ringBoxShape.HalfLength - ringBoxShape.HalfWidth;
                CreateRingWall(simulation, position, ringBoxShape, bodyDescription, heightPerPlatformLevel, radius + wallOffset);
                CreateRingWall(simulation, position, ringBoxShape, bodyDescription, heightPerPlatformLevel, radius - wallOffset);
                CreateRingPlatform(simulation, position + new Vector3(0, heightPerPlatformLevel * ringBoxShape.Height, 0), ringBoxShape, bodyDescription, radius);
                position.Y += heightPerPlatformLevel * ringBoxShape.Height + ringBoxShape.Width;
            }
            return position;
        }

        public unsafe override void Initialize(int threadCount)
        {
            ThreadDispatcher = new ThreadDispatcher(threadCount);
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(90, 1), maximumRecoveryVelocity: 20), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SubsteppingTimestepper(7), 2);

            var ringBoxShape = new Box(0.5f, 1, 3);
            ringBoxShape.ComputeInertia(1, out var ringBoxInertia);
            var boxDescription = BodyDescription.CreateDynamic(new Vector3(), ringBoxInertia, new(Simulation.Shapes.Add(ringBoxShape), 0.1f), new (-0.01f));

            //CreateRingWall(Simulation, default, ringBoxShape, boxDescription, 400, 45);
            var layerPosition = new Vector3();
            const int layerCount = 1;
            var innerRadius = 20f;
            var heightPerPlatform = 10;
            var platformsPerLayer = 30;
            var ringSpacing = 0.5f;
            for (int layerIndex = 0; layerIndex < layerCount; ++layerIndex)
            {
                var ringCount = layerCount - layerIndex;
                for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
                {
                    CreateRing(Simulation, layerPosition, ringBoxShape, boxDescription, innerRadius + ringIndex * (ringBoxShape.Length + ringSpacing) + layerIndex * (ringBoxShape.Length - ringBoxShape.Width), heightPerPlatform, platformsPerLayer);
                }
                layerPosition.Y += platformsPerLayer * (ringBoxShape.Height * heightPerPlatform + ringBoxShape.Width);
            }

            Console.WriteLine($"box count: {Simulation.Bodies.ActiveSet.Count}");
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new (Simulation.Shapes.Add(new Box(500, 1, 500)), 0.1f)));

        }



    }
}
