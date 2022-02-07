using BepuPhysics;
using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using Demos.SpecializedTests;
using System;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Actions;

public class Clonesmash : IAction
{
    float targetTime;
    CameraDirector director;
    public void Initialize(ContentArchive content, Random random, Scene scene)
    {
        var newBodies = new QuickList<BodyDescription>(scene.Simulation.Bodies.ActiveSet.Count, scene.BufferPool);
        for (int setIndex = 0; setIndex < scene.Simulation.Bodies.Sets.Length; ++setIndex)
        {
            ref var set = ref scene.Simulation.Bodies.Sets[setIndex];
            if (set.Allocated)
            {
                for (int bodyIndex = 0; bodyIndex < set.Count; ++bodyIndex)
                {
                    ref var description = ref newBodies.Allocate(scene.BufferPool);
                    set.GetDescription(bodyIndex, out description);
                    //Don't actually want to include any kinematics. Probably won't be any anyway, but just in case.
                    if (description.LocalInertia.InverseMass == 0)
                        --newBodies.Count;
                }
            }
        }
        var regionOfInterest = scene.RegionOfInterest;
        var centerOfInterest = (regionOfInterest.Min + regionOfInterest.Max) * 0.5f;
        var span = regionOfInterest.Max - regionOfInterest.Min;
        var gravityMagnitude = scene.Gravity.Length();
        var gravityDirection = scene.Gravity / gravityMagnitude;
        var gravityOffsetMagnitude = MathF.Abs(Vector3.Dot(span, gravityDirection)) * 2f + 100;
        var offset = -gravityOffsetMagnitude * gravityDirection;
        var pose = TestHelpers.CreateRandomPose(random, default);
        for (int i = 0; i < newBodies.Count; ++i)
        {
            ref var newBody = ref newBodies[i];
            newBody.Pose.Position = centerOfInterest + offset + QuaternionEx.Transform(newBody.Pose.Position - centerOfInterest, pose.Orientation);
            newBody.Pose.Orientation = QuaternionEx.Concatenate(newBody.Pose.Orientation, pose.Orientation);
            scene.Simulation.Bodies.Add(newBody);
        }

        //d = 1/2 * a * t^2
        //sqrt(d * 2 / a)
        targetTime = MathF.Sqrt(gravityOffsetMagnitude * 2 / gravityMagnitude) + 1.5f * MathF.Sqrt(span.Length() * 2 / gravityMagnitude);
        director = new CameraDirector(new ICameraController[]
        {
            new RotatingCamera(MathF.PI * 0.15f, (float)random.NextDouble() * MathF.PI * 2, 0.03f, 4, 0.5f),
        }, random);
    }


    public bool Update(Scene scene, Random random, Camera camera, float accumulatedTime, float accumulatedRealTime, bool controlCamera)
    {
        if (controlCamera)
            director.Update(scene, camera, random, accumulatedTime, accumulatedRealTime);
        return accumulatedTime < targetTime;
    }
}

