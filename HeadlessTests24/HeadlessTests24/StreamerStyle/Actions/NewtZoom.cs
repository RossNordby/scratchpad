using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using Demos;
using SharpDX.Mathematics.Interop;
using System;
using System.Numerics;


namespace HeadlessTests24.StreamerStyle.Actions;

public class NewtZoom : IAction
{
    float targetTime;
    CameraDirector director;

    public void Initialize(ContentArchive content, Random random, Scene scene)
    {
        var newtCount = random.Next(1, 6);

        var span = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;
        DemoMeshHelper.LoadModel(content, scene.BufferPool, @"Content\newt.obj", Vector3.One, out var templateMesh);

        var longestTime = 0f;

        for (int i = 0; i < newtCount; ++i)
        {
            var target = scene.RegionOfInterest.Min * new Vector3(1, 0, 1) + span * new Vector3((float)random.NextDouble(), 0, (float)random.NextDouble());
            var theta = (float)random.NextDouble() * 2 * MathF.PI;
            var newtStartDistance = span.Length() + 200 + (float)random.NextDouble() * 300;
            var start = target + newtStartDistance * new Vector3(MathF.Sin(theta), 0, MathF.Cos(theta));

            var newtVelocity = 100 + (float)random.NextDouble() * 100;
            var t = newtStartDistance / newtVelocity;
            if (t > longestTime)
                longestTime = t;
            var mesh = templateMesh;
            //sometimes, a very large newt.
            mesh.Scale = new Vector3(random.NextDouble() < 0.02 ? 50 + 100 * (float)random.NextDouble() : 5 + 10 * (float)random.NextDouble());

            //sometimes, a backward newt
            var orientation = QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, random.NextDouble() < 0.02 ? theta : theta + MathF.PI);

            scene.Simulation.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(start, orientation), new BodyVelocity(Vector3.Normalize(target - start) * newtVelocity), scene.Simulation.Shapes.Add(mesh), -1));
        }

        targetTime = 5 + 2 * longestTime;
        director = new CameraDirector(new ICameraController[]
        {
            new RotatingCamera(MathF.PI * 0.15f, (float)random.NextDouble() * MathF.PI * 2, 0f, 4, 0.5f, 0.8f),
        }, random);
    }


    public bool Update(Scene scene, Random random, Camera camera, float accumulatedTime, float accumulatedRealTime, bool controlCamera)
    {
        if (controlCamera)
            director.Update(scene, camera, random, accumulatedTime, accumulatedRealTime);
        return accumulatedTime < targetTime;
    }
}

