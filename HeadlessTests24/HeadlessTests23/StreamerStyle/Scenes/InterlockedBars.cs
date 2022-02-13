using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using System.Numerics;

namespace HeadlessTests23.StreamerStyle.Scenes;
public class InterlockedBars : Scene
{
    public static void GetSimulationPropertiesForDependencyChain(int dependencyChainLength, out int substepCount, out int hz, out int solverIterationCount)
    {
        substepCount = Math.Max(1, dependencyChainLength / 60);
        hz = 30 + 10 * (substepCount - 1);
        solverIterationCount = Math.Min(8, Math.Max(2, (int)MathF.Ceiling(8 / (substepCount * 0.8f))));
    }

    public static void CreateCell(Simulation simulation, in RigidPose worldPose, Vector3 cellMin, float cellInset, int heightInPlanks, bool intermediateCell, Box plankShape, BodyDescription plankDescription, Box floorShape, BodyDescription floorDescription)
    {
        for (int plankIndex = 0; plankIndex < heightInPlanks; ++plankIndex)
        {
            if ((plankIndex & 1) == 0)
            {
                //Rotate by 90 degrees to align along the X axis.
                QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, MathF.PI * 0.5f, out plankDescription.Pose.Orientation);
                QuaternionEx.Concatenate(plankDescription.Pose.Orientation, worldPose.Orientation, out plankDescription.Pose.Orientation);

                plankDescription.Pose.Position = cellMin + new Vector3(plankShape.HalfLength, plankShape.HalfHeight + plankIndex * plankShape.Height, cellInset + plankShape.HalfWidth);
                RigidPose.Transform(plankDescription.Pose.Position, worldPose, out plankDescription.Pose.Position);
                simulation.Bodies.Add(plankDescription);

                plankDescription.Pose.Position = cellMin + new Vector3(plankShape.HalfLength, plankShape.HalfHeight + plankIndex * plankShape.Height, plankShape.Length - cellInset - plankShape.HalfWidth);
                RigidPose.Transform(plankDescription.Pose.Position, worldPose, out plankDescription.Pose.Position);
                simulation.Bodies.Add(plankDescription);
            }
            else
            {
                //Aligned along the Z axis.
                plankDescription.Pose.Orientation = worldPose.Orientation;

                plankDescription.Pose.Position = cellMin + new Vector3(cellInset + plankShape.HalfWidth, plankShape.HalfHeight + plankIndex * plankShape.Height, plankShape.HalfLength);
                RigidPose.Transform(plankDescription.Pose.Position, worldPose, out plankDescription.Pose.Position);
                simulation.Bodies.Add(plankDescription);

                plankDescription.Pose.Position = cellMin + new Vector3(plankShape.Length - cellInset - plankShape.HalfWidth, plankShape.HalfHeight + plankIndex * plankShape.Height, plankShape.HalfLength);
                RigidPose.Transform(plankDescription.Pose.Position, worldPose, out plankDescription.Pose.Position);
                simulation.Bodies.Add(plankDescription);
            }
        }
        if (!intermediateCell)
        {
            //Main cell, so it needs a floor on top.
            floorDescription.Pose.Position = cellMin + new Vector3(plankShape.Length * 0.5f, plankShape.Height * heightInPlanks + floorShape.HalfHeight, plankShape.Length * 0.5f);
            floorDescription.Pose.Orientation = worldPose.Orientation;
            RigidPose.Transform(floorDescription.Pose.Position, worldPose, out floorDescription.Pose.Position);
            simulation.Bodies.Add(floorDescription);
        }
    }
    public static void CreateFloor(Simulation simulation, RigidPose worldPose, in Vector3 localPosition, int planksPerFloor, int widthInCells, int lengthInCells, float cellSpacing, float cellInset,
        Box plankShape, BodyDescription plankDescription, Box floorShape, BodyDescription floorDescription)
    {
        var localMin = localPosition - 0.5f * new Vector3(widthInCells * plankShape.Length + (widthInCells - 1) * cellSpacing, 0, lengthInCells * plankShape.Length + (lengthInCells - 1) * cellSpacing);
        for (int cellX = 0; cellX < widthInCells; ++cellX)
        {
            for (int cellZ = 0; cellZ < lengthInCells; ++cellZ)
            {
                var cellMin = localMin + new Vector3(cellX, 0, cellZ) * (plankShape.Length + cellSpacing);
                CreateCell(simulation, worldPose, cellMin, cellInset, planksPerFloor, false, plankShape, plankDescription, floorShape, floorDescription);
            }
        }
        //Intermediates.
        for (int cellX = 0; cellX < widthInCells - 1; ++cellX)
        {
            for (int cellZ = 0; cellZ < lengthInCells - 1; ++cellZ)
            {
                var cellMin = localMin + new Vector3(cellX + 0.5f, 0, cellZ + 0.5f) * (plankShape.Length + cellSpacing);
                CreateCell(simulation, worldPose, cellMin, cellInset, planksPerFloor, true, plankShape, plankDescription, floorShape, floorDescription);
            }
        }
    }
    public static void CreateBuilding(Simulation simulation, RigidPose worldPose, int floors, int planksPerFloor, int widthInCells, int lengthInCells, float cellSpacing, float cellInset,
        Box plankShape, BodyDescription plankDescription, Box floorShape, BodyDescription floorDescription)
    {
        for (int i = 0; i < floors; ++i)
        {
            var localPosition = i * new Vector3(0, plankShape.Height * planksPerFloor + floorShape.Height, 0);
            CreateFloor(simulation, worldPose, localPosition, planksPerFloor, widthInCells, lengthInCells, cellSpacing, cellInset, plankShape, plankDescription, floorShape, floorDescription);
        }
    }

    public static void CreateFloor(Simulation simulation, RigidPose worldPose, in Vector3 localPosition, int planksPerFloor, float cellSpacing, float cellInset,
        Box plankShape, BodyDescription plankDescription, Box floorShape, BodyDescription floorDescription,
        bool[,] cellMask)
    {
        var localMin = localPosition - 0.5f * new Vector3(cellMask.GetLength(0) * plankShape.Length + (cellMask.GetLength(0) - 1) * cellSpacing, 0, cellMask.GetLength(1) * plankShape.Length + (cellMask.GetLength(1) - 1) * cellSpacing);
        for (int cellX = 0; cellX < cellMask.GetLength(0); ++cellX)
        {
            for (int cellZ = 0; cellZ < cellMask.GetLength(1); ++cellZ)
            {
                if (cellMask[cellX, cellZ])
                {
                    var cellMin = localMin + new Vector3(cellX, 0, cellZ) * (plankShape.Length + cellSpacing);
                    CreateCell(simulation, worldPose, cellMin, cellInset, planksPerFloor, false, plankShape, plankDescription, floorShape, floorDescription);
                }
            }
        }
        //Intermediates.
        for (int cellX = 0; cellX < cellMask.GetLength(0) - 1; ++cellX)
        {
            for (int cellZ = 0; cellZ < cellMask.GetLength(1) - 1; ++cellZ)
            {
                var neighborCount = (cellMask[cellX, cellZ] ? 1 : 0) + (cellMask[cellX + 1, cellZ] ? 1 : 0) + (cellMask[cellX, cellZ + 1] ? 1 : 0) + (cellMask[cellX + 1, cellZ + 1] ? 1 : 0);
                var diagonal = (cellMask[cellX, cellZ] && cellMask[cellX + 1, cellZ + 1]) || (cellMask[cellX + 1, cellZ] && cellMask[cellX, cellZ + 1]);
                if (neighborCount > 2 || diagonal)
                {
                    var cellMin = localMin + new Vector3(cellX + 0.5f, 0, cellZ + 0.5f) * (plankShape.Length + cellSpacing);
                    CreateCell(simulation, worldPose, cellMin, cellInset, planksPerFloor, true, plankShape, plankDescription, floorShape, floorDescription);
                }
            }
        }
    }

    public static int CreateBuilding(Simulation simulation, RigidPose worldPose, int maxFloors, int planksPerFloor, float cellSpacing, float cellInset,
        Box plankShape, BodyDescription plankDescription, Box floorShape, BodyDescription floorDescription,
        bool[,] cellMask, Random random, Func<bool[,], Random, bool[,]> maskModifier)
    {
        for (int i = 0; i < maxFloors; ++i)
        {
            var localPosition = i * new Vector3(0, plankShape.Height * planksPerFloor + floorShape.Height, 0);
            CreateFloor(simulation, worldPose, localPosition, planksPerFloor, cellSpacing, cellInset, plankShape, plankDescription, floorShape, floorDescription, cellMask);
            cellMask = maskModifier(cellMask, random);
            bool allMaskedOut = true;
            for (int cellX = 0; cellX < cellMask.GetLength(0) - 1; ++cellX)
            {
                for (int cellZ = 0; cellZ < cellMask.GetLength(1) - 1; ++cellZ)
                {
                    if (cellMask[cellX, cellZ])
                    {
                        allMaskedOut = false;
                    }
                }
            }
            if (allMaskedOut)
                return i + 1;
        }
        return maxFloors;
    }

    static bool IntermediateExistsForMin(int x, int z, bool[,] cellMask)
    {
        if (x < 0 || x >= cellMask.GetLength(0) - 1 || z < 0 || z >= cellMask.GetLength(1) - 1)
            return false;
        return cellMask[x, z] && cellMask[x + 1, z] && cellMask[x, z + 1] && cellMask[x + 1, z + 1];
    }
    static int CountIntermediatesAroundCell(int x, int z, bool[,] cellMask)
    {
        int count = 0;
        if (IntermediateExistsForMin(x, z, cellMask))
            ++count;
        if (IntermediateExistsForMin(x - 1, z, cellMask))
            ++count;
        if (IntermediateExistsForMin(x, z - 1, cellMask))
            ++count;
        if (IntermediateExistsForMin(x - 1, z - 1, cellMask))
            ++count;
        return count;
    }
    static bool[,] ModifyMask(bool[,] cellMask, Random random)
    {
        var nextMask = new bool[cellMask.GetLength(0), cellMask.GetLength(1)];
        for (int i = 0; i < cellMask.GetLength(0); ++i)
        {
            for (int j = 0; j < cellMask.GetLength(1); ++j)
            {
                nextMask[i, j] = cellMask[i, j];
            }
        }
        for (int i = 0; i < cellMask.GetLength(0); ++i)
        {
            for (int j = 0; j < cellMask.GetLength(1); ++j)
            {

                int adjacentEmptyCount = 0;
                for (int a = i - 1; a <= i + 1; ++a)
                {
                    for (int b = j - 1; b <= j + 1; ++b)
                    {
                        if (a < 0 || b < 0 || a >= cellMask.GetLength(0) || b >= cellMask.GetLength(1) || !cellMask[a, b])
                        {
                            ++adjacentEmptyCount;
                        }
                    }
                }
                var intermediatesCount = CountIntermediatesAroundCell(i, j, cellMask);
                var threshold = intermediatesCount switch { 0 => 1, 1 => 0.3f, 2 => 0.15, 3 => 0.02, _ => 0 };
                if (random.NextDouble() < threshold)
                {
                    nextMask[i, j] = false;
                }
            }
        }
        return nextMask;
    }

    public override float TimestepDuration => 1 / 60f;
    public override Vector3 Gravity => new Vector3(0, -10, 0);

    public static int CreateProceduralBuilding(int widthInCells, int lengthInCells, Random random, Simulation simulation, RigidPose pose, int maxFloors, int planksPerFloor, float cellSpacing, float cellInset,
        Box plankShape, BodyDescription plankDescription, Box floorShape, BodyDescription floorDescription)
    {
        bool[,] cellMask = new bool[widthInCells, lengthInCells];
        var blockerCount = random.Next(3, 6);
        Span<(int x, int y, int radius, int boundsType)> blockers = stackalloc (int, int, int, int)[blockerCount];
        for (int i = 0; i < blockerCount; ++i)
        {
            ref var blocker = ref blockers[i];
            blocker.x = random.Next(0, widthInCells);
            blocker.y = random.Next(0, lengthInCells);
            blocker.radius = (int)((0.1 + 0.2 * random.NextDouble()) * Math.Max(widthInCells, lengthInCells));
            blocker.boundsType = random.Next(0, 3);
        }

        for (int i = 0; i < cellMask.GetLength(0); ++i)
        {
            for (int j = 0; j < cellMask.GetLength(1); ++j)
            {
                var blocked = false;
                for (int k = 0; k < blockers.Length; ++k)
                {
                    ref var blocker = ref blockers[k];
                    var offsetX = i - blocker.x;
                    var offsetY = j - blocker.y;
                    blocked = blocker.boundsType switch
                    {
                        0 => offsetX * offsetX + offsetY * offsetY < blocker.radius * blocker.radius,
                        1 => Math.Abs(offsetX) + Math.Abs(offsetY) < blocker.radius,
                        _ => Math.Abs(offsetX) < blocker.radius && Math.Abs(offsetY) < blocker.radius
                    };
                    if (blocked)
                        break;
                }
                cellMask[i, j] = !blocked;
            }
        }
        var actualFloorCount = CreateBuilding(simulation, pose, maxFloors, planksPerFloor, cellSpacing, cellInset, plankShape, plankDescription, floorShape, floorDescription, cellMask, random, ModifyMask);
        return actualFloorCount;
    }

    public override void Initialize(Random random, int threadCount)
    {
        ThreadDispatcher = new ThreadDispatcher(threadCount);
        var floors = random.Next(20, 50);
        var planksPerFloor = random.Next(2, 5);
        var cellInset = 0f;
        var cellSpacing = 0.125f;
        var areaRand = random.NextDouble();
        var area = 24 * 24 + (int)(24 * 24 * areaRand * areaRand * areaRand * areaRand * areaRand);
        var widthInCells = random.Next(24, 24 + 24);
        var lengthInCells = area / widthInCells;
        //var widthRand = random.NextDouble();
        //var lengthRand = random.NextDouble();
        //var widthInCells = 24 + (int)(36 * widthRand * widthRand * widthRand);
        //var lengthInCells = 24 + (int)(36 * lengthRand * lengthRand * lengthRand);

        var dependencyChainLength = floors * (planksPerFloor + 1);
        GetSimulationPropertiesForDependencyChain(dependencyChainLength, out var substepCount, out int hz, out int solverIterationCount);

        Simulation = Simulation.Create(BufferPool, new NarrowPhaseCallbacks() { SpringSettings = new SpringSettings(hz, 1) }, new DemoPoseIntegratorCallbacks(Gravity), new SolveDescription(solverIterationCount, substepCount));

        var plankShape = new Box(0.75f, 1.75f, 4);
        var plankInertia = plankShape.ComputeInertia(1);
        var plankDescription = BodyDescription.CreateDynamic(new Vector3(), plankInertia, new(Simulation.Shapes.Add(plankShape), 0.1f), -0.01f);
        var floorShape = new Box(plankShape.Length + cellSpacing * 0.9f, 0.25f, plankShape.Length + cellSpacing * 0.9f);
        var floorInertia = floorShape.ComputeInertia(1);
        var floorDescription = BodyDescription.CreateDynamic(new Vector3(), floorInertia, new(Simulation.Shapes.Add(floorShape), 0.1f), -0.01f);


        //CreateBuilding(Simulation, new RigidPose(new Vector3(10, 0, 0), QuaternionEx.Identity), floors, planksPerFloor, widthInCells, lengthInCells, cellSpacing, cellInset, plankShape, plankDescription, floorShape, floorDescription);
        var actualFloorCount = CreateProceduralBuilding(widthInCells, lengthInCells, random, Simulation, new RigidPose(new Vector3(10, 0, 0), QuaternionEx.Identity), floors, planksPerFloor, cellSpacing, cellInset, plankShape, plankDescription, floorShape, floorDescription);
        GetSimulationPropertiesForDependencyChain(actualFloorCount * (planksPerFloor + 1), out substepCount, out hz, out solverIterationCount);
        (Simulation.NarrowPhase as NarrowPhase<NarrowPhaseCallbacks>).Callbacks.SpringSettings.Frequency = hz;
        Simulation.Solver.VelocityIterationCount = solverIterationCount;
        Simulation.Solver.SubstepCount = substepCount;

        CreateRegionOfInterest();

        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 0), Simulation.Shapes.Add(new Box(2500, 10, 2500))));
    }



    public override void Update()
    {
        Simulation.Timestep(TimestepDuration, ThreadDispatcher);
    }

}
