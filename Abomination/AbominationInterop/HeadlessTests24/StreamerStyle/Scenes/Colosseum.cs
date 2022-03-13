using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Scenes;
public class Colosseum : Scene
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
                bodyDescription.Pose = new RigidPose(
                    position + new Vector3(-MathF.Cos(angle) * radius, (ringIndex + 0.5f) * ringBoxShape.Height, MathF.Sin(angle) * radius),
                    QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle));
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
            bodyDescription.Pose = new RigidPose(
                position + new Vector3(-MathF.Cos(angle) * radius, ringBoxShape.HalfWidth, MathF.Sin(angle) * radius),
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

    public static void CreateColosseum(Simulation simulation, Vector3 position, Box ringBoxShape, BodyDescription boxDescription, float innerRadius, float ringSpacing, int layerCount, int platformsPerLayer, int heightPerPlatform)
    {
        for (int layerIndex = 0; layerIndex < layerCount; ++layerIndex)
        {
            var ringCount = layerCount - layerIndex;
            for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
            {
                CreateRing(simulation, position, ringBoxShape, boxDescription, innerRadius + ringIndex * (ringBoxShape.Length + ringSpacing) + layerIndex * (ringBoxShape.Length - ringBoxShape.Width), heightPerPlatform, platformsPerLayer);
            }
            position.Y += platformsPerLayer * (ringBoxShape.Height * heightPerPlatform + ringBoxShape.Width);
        }
    }
    public override float TimestepDuration => 1 / 60f;
    public override Vector3 Gravity => new Vector3(0, -10, 0);

    public override void Initialize(Random random, int threadCount)
    {
        ThreadDispatcher = new ThreadDispatcher(threadCount);

        var layerRand = random.NextDouble();
        var heightPerRand = random.NextDouble();
        var platformsPerRand = random.NextDouble();
        var radiusRand = random.NextDouble();
        var heightPerPlatform = 2 + (int)(heightPerRand * heightPerRand * 5);
        var platformsPerLayer = 2 + (int)(platformsPerRand * platformsPerRand * 5);
        var layerCount = 3 + (int)(6 * layerRand * layerRand);
        var innerRadius = 3.5f + (float)(radiusRand * radiusRand * MathF.Max(0, 50 - layerCount * 6));

        //layerCount = 3;
        //heightPerPlatform = 2;
        //platformsPerLayer = 2;
        //innerRadius = 5;

        var dependencyChainLength = (1 + heightPerPlatform) * platformsPerLayer * layerCount;
        InterlockedBars.GetSimulationPropertiesForDependencyChain(dependencyChainLength, out var substepCount, out var hz, out var solverIterationCount);
        Simulation = Simulation.Create(BufferPool, new NarrowPhaseCallbacks() { SpringSettings = new SpringSettings(hz, 1) }, new DemoPoseIntegratorCallbacks(Gravity), new SolveDescription(solverIterationCount, substepCount));

        var ringBoxShape = new Box(0.5f, 1, 3);
        var ringBoxInertia = ringBoxShape.ComputeInertia(1);
        var boxDescription = BodyDescription.CreateDynamic(new Vector3(), ringBoxInertia, new(Simulation.Shapes.Add(ringBoxShape), 0.1f), -0.01f);

        var layerPosition = new Vector3();
        var ringSpacing = 0.5f;
        CreateColosseum(Simulation, layerPosition, ringBoxShape, boxDescription, innerRadius, ringSpacing, layerCount, platformsPerLayer, heightPerPlatform);

        CreateRegionOfInterest();

        //Console.WriteLine($"box count: {Simulation.Bodies.ActiveSet.Count}");
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 0), Simulation.Shapes.Add(new Box(2500, 10, 2500))));
    }

    public override void Update()
    {
        Simulation.Timestep(TimestepDuration, ThreadDispatcher);
    }
}

