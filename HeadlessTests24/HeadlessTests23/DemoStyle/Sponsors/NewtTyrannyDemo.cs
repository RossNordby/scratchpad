﻿using System.Numerics;
using BepuPhysics;
using BepuUtilities.Collections;
using BepuPhysics.Collidables;
using HeadlessTests23.DemoStyle.Characters;
using BepuUtilities;

namespace HeadlessTests23.DemoStyle.Sponsors;

public class NewtTyrannyDemo : Demo
{
    QuickList<SponsorNewt> newts;

    Vector2 newtArenaMin, newtArenaMax;
    Random random;
    CharacterControllers characterControllers;
    QuickList<SponsorCharacterAI> characterAIs;
    public override void Initialize(int threadCount)
    {
        ThreadDispatcher = new ThreadDispatcher(threadCount);

        characterControllers = new CharacterControllers(BufferPool);
        Simulation = Simulation.Create(BufferPool, new CharacterNarrowphaseCallbacks(characterControllers), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper(), 8);
        Simulation.Deterministic = true;
        Mesh newtMesh;
        using (var stream = File.Open(@"Content\newt.obj", FileMode.Open))
        {
            newtMesh = MeshLoader.LoadMesh(stream, Vector3.One, BufferPool);
        }
        var newtShape = Simulation.Shapes.Add(newtMesh);
        var newtCount = 10;
        newts = new QuickList<SponsorNewt>(newtCount, BufferPool);
        newtArenaMin = new Vector2(-250);
        newtArenaMax = new Vector2(250);
        random = new Random(8);
        for (int i = 0; i < newtCount; ++i)
        {
            ref var newt = ref newts.AllocateUnsafely();
            newt = new SponsorNewt(Simulation, newtShape, 0, newtArenaMin, newtArenaMax, random, i);
        }

        const float floorSize = 520;
        const float wallThickness = 200;
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10f, 0), new(Simulation.Shapes.Add(new Box(floorSize, 20, floorSize)), 0.1f)));
        Simulation.Statics.Add(new StaticDescription(new Vector3(floorSize * -0.5f - wallThickness * 0.5f, -5, 0), new(Simulation.Shapes.Add(new Box(wallThickness, 30, floorSize + wallThickness * 2)), 0.1f)));
        Simulation.Statics.Add(new StaticDescription(new Vector3(floorSize * 0.5f + wallThickness * 0.5f, -5, 0), new(Simulation.Shapes.Add(new Box(wallThickness, 30, floorSize + wallThickness * 2)), 0.1f)));
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, floorSize * -0.5f - wallThickness * 0.5f), new(Simulation.Shapes.Add(new Box(floorSize, 30, wallThickness)), 0.1f)));
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, floorSize * 0.5f + wallThickness * 0.5f), new(Simulation.Shapes.Add(new Box(floorSize, 30, wallThickness)), 0.1f)));

        const int characterCount = 2000;
        characterAIs = new QuickList<SponsorCharacterAI>(characterCount, BufferPool);
        var characterCollidable = Simulation.Shapes.Add(new Capsule(0.5f, 1f));
        for (int i = 0; i < characterCount; ++i)
        {
            var position2D = newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2(random.NextSingle(), random.NextSingle());
            var targetPosition = 0.5f * (newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2(random.NextSingle(), random.NextSingle()));
            characterAIs.AllocateUnsafely() = new SponsorCharacterAI(characterControllers, new(characterCollidable, float.MaxValue), new Vector3(position2D.X, 5, position2D.Y), targetPosition);
        }

        const int hutCount = 120;
        var hutBoxShape = new Box(0.4f, 2, 3);
        hutBoxShape.ComputeInertia(20, out var hutBoxInertia);
        var obstacleDescription = BodyDescription.CreateDynamic(new RigidPose(new Vector3()), hutBoxInertia, new CollidableDescription(Simulation.Shapes.Add(hutBoxShape), 0.1f), new(1e-2f));

        for (int i = 0; i < hutCount; ++i)
        {
            var position2D = newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2(random.NextSingle(), random.NextSingle());
            Colosseum24VideoDemo.CreateRing(Simulation, new Vector3(position2D.X, 0, position2D.Y), hutBoxShape, obstacleDescription, 4 + random.NextSingle() * 8, 2, random.Next(1, 10));

        }

        var overlordNewtShape = newtMesh;
        overlordNewtShape.Scale = new Vector3(60, 60, 60);
        Simulation.Statics.Add(new StaticDescription(new Vector3(0, 10, -floorSize * 0.5f - 70), new (Simulation.Shapes.Add(overlordNewtShape), 0.1f)));


        character = new CharacterInput(characterControllers, new Vector3(-108.89504f, 28.403418f, 38.27505f), new Capsule(0.5f, 1), 0.1f, .1f, 20, 100, 6, 4, MathF.PI * 0.4f);
    }

    CharacterInput character;

    double simulationTime;
    public override void Update()
    {
        Simulation.Timestep(TimestepDuration, ThreadDispatcher);
        for (int i = 0; i < newts.Count; ++i)
        {
            newts[i].Update(Simulation, simulationTime, 0, newtArenaMin, newtArenaMax, random, 1f / TimestepDuration);
        }
        for (int i = 0; i < characterAIs.Count; ++i)
        {
            characterAIs[i].Update(characterControllers, Simulation, ref newts, newtArenaMin, newtArenaMax, random);
        }
        simulationTime += TimestepDuration;
    }

}
