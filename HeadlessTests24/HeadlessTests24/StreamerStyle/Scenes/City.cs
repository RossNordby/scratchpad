using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Scenes;

public class City : Scene
{
    static bool CanPlace(Span<BoundingBox> previousBounds, int buildingIndex, float spawnRegionWidth, Vector3 min, Vector3 max, Random random, out Vector3 position)
    {
        Vector3 positionCandidate;
        int attemptCount = 0;
        bool blocked;
        do
        {
            positionCandidate = spawnRegionWidth * new Vector3((float)random.NextDouble() - 0.5f, 0, (float)random.NextDouble() - 0.5f);
            ref var boundsCandidate = ref previousBounds[buildingIndex];
            boundsCandidate.Min = positionCandidate + min;
            boundsCandidate.Max = positionCandidate + max;
            blocked = false;
            for (int j = 0; j < buildingIndex; ++j)
            {
                if (BoundingBox.Intersects(previousBounds[j], boundsCandidate))
                {
                    blocked = true;
                    break;
                }
            }
            ++attemptCount;
        } while (blocked && attemptCount < 1000);
        if (blocked)
        {
            previousBounds[buildingIndex] = new BoundingBox { Min = new Vector3(float.MaxValue), Max = new Vector3(float.MinValue) };
            position = default;
            return false;
        }
        position = positionCandidate;
        return true;
    }

    public override float TimestepDuration => 1 / 60f;
    public override Vector3 Gravity => new Vector3(0, -10, 0);

    public override void Initialize(Random random, int threadCount)
    {
        ThreadDispatcher = new ThreadDispatcher(threadCount);
        Simulation = Simulation.Create(BufferPool, new NarrowPhaseCallbacks() { SpringSettings = new SpringSettings(30, 1) }, new DemoPoseIntegratorCallbacks(Gravity), new SolveDescription(8, 1));

        var ringBoxShape = new Box(0.5f, 1, 3);
        var ringBoxInertia = ringBoxShape.ComputeInertia(1);
        var boxDescription = BodyDescription.CreateDynamic(new Vector3(), ringBoxInertia, new(Simulation.Shapes.Add(ringBoxShape), 0.1f), 0.01f);
        const float ringSpacing = 0.5f;

        var cellInset = 0f;
        var cellSpacing = 0.125f;
        var plankShape = new Box(0.75f, 1.25f, 4);
        var plankInertia = plankShape.ComputeInertia(1);
        var plankDescription = BodyDescription.CreateDynamic(new Vector3(), plankInertia, new(Simulation.Shapes.Add(plankShape), 0.1f), 0.01f);
        var floorShape = new Box(plankShape.Length + cellSpacing * 0.9f, 0.25f, plankShape.Length + cellSpacing * 0.9f);
        var floorInertia = floorShape.ComputeInertia(1);
        var floorDescription = BodyDescription.CreateDynamic(new Vector3(), floorInertia, new(Simulation.Shapes.Add(floorShape), 0.1f), 0.01f);


        var longestDependencyChainLength = 0;
        var buildingCount = random.Next(4, 80);
        var area = buildingCount * 50 * 50;
        var spawnRegionWidth = MathF.Sqrt(area);
        Span<BoundingBox> previousBounds = stackalloc BoundingBox[buildingCount];
        int failedCount = 0;
        for (int i = 0; i < buildingCount; ++i)
        {
            if (random.NextDouble() < 0.3f)
            {
                //Colosseum.
                var layerRand = random.NextDouble();
                var layerCount = 1 + (int)(2 * layerRand * layerRand);
                var innerRadius = 4f + 4 * (float)random.NextDouble();
                var heightPerPlatform = random.Next(3, 8);
                var platformsPerLayer = random.Next(3, 8);
                var dependencyChainLength = (1 + heightPerPlatform) * platformsPerLayer * layerCount;
                if (dependencyChainLength > longestDependencyChainLength)
                {
                    longestDependencyChainLength = dependencyChainLength;
                }
                var max = new Vector3(innerRadius + layerCount * (ringBoxShape.Length + ringSpacing));
                max.Y = 0;
                var min = -max;

                if (CanPlace(previousBounds, i, spawnRegionWidth, min, max, random, out var position))
                {
                    Colosseum.CreateColosseum(Simulation, position, ringBoxShape, boxDescription, innerRadius, ringSpacing, layerCount, platformsPerLayer, heightPerPlatform);
                }
                else
                {
                    ++failedCount;
                }
            }
            else
            {
                var floors = random.Next(10, 15);
                var planksPerFloor = random.Next(2, 5);
                var widthRand = random.NextDouble();
                var lengthRand = random.NextDouble();
                var areaRand = random.NextDouble();
                var buildingArea = 6 * 6 + (int)(6 * 6 * areaRand * areaRand * areaRand * areaRand * areaRand);
                var widthInCells = random.Next(6, 6 + 6);
                var lengthInCells = buildingArea / widthInCells;
                var max = new Vector3(widthInCells, 0, lengthInCells) * (plankShape.Length + cellSpacing);
                var min = -max;

                var dependencyChainLength = floors * (planksPerFloor + 1);
                if (dependencyChainLength > longestDependencyChainLength)
                {
                    longestDependencyChainLength = dependencyChainLength;
                }

                //Interlocked.           
                if (CanPlace(previousBounds, i, spawnRegionWidth, min, max, random, out var position))
                {
                    InterlockedBars.CreateProceduralBuilding(widthInCells, lengthInCells, random, Simulation, new RigidPose(position), floors, planksPerFloor, cellSpacing, cellInset, plankShape, plankDescription, floorShape, floorDescription);
                }
                else
                {
                    ++failedCount;
                }
            }
        }
        Console.WriteLine($"failed placement count: {failedCount}");

        InterlockedBars.GetSimulationPropertiesForDependencyChain(longestDependencyChainLength, out var substepCount, out var hz, out var solverIterationCount);
        (Simulation.NarrowPhase as NarrowPhase<NarrowPhaseCallbacks>).Callbacks.SpringSettings.Frequency = hz;
        Simulation.Solver.VelocityIterationCount = solverIterationCount;
        Simulation.Solver.SubstepCount = substepCount;


        CreateRegionOfInterest();

        //Console.WriteLine($"box count: {Simulation.Bodies.ActiveSet.Count}");
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 0), Simulation.Shapes.Add(new Box(3500, 10, 3500))));
    }

    public override void Update()
    {
        Simulation.Timestep(TimestepDuration, ThreadDispatcher);
    }

}
