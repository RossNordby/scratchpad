using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using Demos.Demos;
using Demos.SpecializedTests;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Security;

namespace HeadlessTests24.StreamerStyle.Actions;

public class Ragdolls : IAction
{
    float targetTime;
    QuickList<BodyHandle> hipHandles;
    CameraDirector director;
    public unsafe void Initialize(ContentArchive content, Random random, Scene scene)
    {
        var rand = random.NextDouble();
        var count = 150 + (int)(130 * rand * rand);
        hipHandles = new QuickList<BodyHandle>(count, scene.BufferPool);

        var sceneSpan = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;
        var targetMin = scene.RegionOfInterest.Min + sceneSpan * 0.2f;
        var targetSpan = sceneSpan * 0.6f;
        var distance = 50 + sceneSpan.Length() * 1f;
        var velocityMagnitude = 30 + 100 * (float)random.NextDouble();

        var longestTimeUntilTarget = 0f;

        //We need filters for the ragdolls.
        ref var filters = ref (scene.Simulation.NarrowPhase as NarrowPhase<NarrowPhaseCallbacks>).Callbacks.Filters;
        filters = new CollidableProperty<Demos.Demos.SubgroupCollisionFilter>(scene.Simulation);
        for (int setIndex = 0; setIndex < scene.Simulation.Bodies.Sets.Length; ++setIndex)
        {
            ref var set = ref scene.Simulation.Bodies.Sets[setIndex];
            if (set.Allocated)
            {
                for (int i = 0; i < set.Count; ++i)
                {
                    filters[set.IndexToHandle[i]] = new Demos.Demos.SubgroupCollisionFilter(0);
                }
            }
        }
        RagdollDemo.MassMultiplier = 0.1f;
        RagdollDemo.SleepThreshold = -1f;
        int groupIndex = 1;
        for (int i = 0; i < count; ++i)
        {
            var target = targetMin + targetSpan * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
            var yaw = (float)random.NextDouble() * MathF.PI * 2;
            var offsetDirection = new Vector3(MathF.Sin(yaw), 1f, MathF.Cos(yaw)) * .707f;
            var offset = offsetDirection * distance;
            var start = target + offset;
            var timeUntilTarget = new Vector2(offset.X, offset.Z).Length() / (new Vector2(offsetDirection.X, offsetDirection.Z).Length() * velocityMagnitude);
            if (timeUntilTarget > longestTimeUntilTarget)
                longestTimeUntilTarget = timeUntilTarget;

            //d = v * t + 0.5 * a * t^2
            //(d - 0.5 * a * t^2) / t = v
            var velocity = new Vector3(-offsetDirection.X * velocityMagnitude, (-offset.Y - 0.5f * scene.Gravity.Y * timeUntilTarget * timeUntilTarget) / timeUntilTarget, -offsetDirection.Z * velocityMagnitude);

            var orientation = TestHelpers.CreateRandomPose(random, default).Orientation;
            var ragdollHandles = Demos.Demos.RagdollDemo.AddRagdoll(start, orientation, groupIndex++, filters, scene.Simulation);
            //This could be done better, but...  ... .... ..........
            new BodyReference(ragdollHandles.Hips, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.Abdomen, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.Chest, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.Head, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.LeftArm.UpperArm, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.LeftArm.LowerArm, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.LeftArm.Hand, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.RightArm.UpperArm, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.RightArm.LowerArm, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.RightArm.Hand, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.LeftLeg.UpperLeg, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.LeftLeg.LowerLeg, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.LeftLeg.Foot, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.RightLeg.UpperLeg, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.RightLeg.LowerLeg, scene.Simulation.Bodies).Velocity.Linear = velocity;
            new BodyReference(ragdollHandles.RightLeg.Foot, scene.Simulation.Bodies).Velocity.Linear = velocity;
            hipHandles.AllocateUnsafely() = ragdollHandles.Hips;
        }

        targetTime = 10 + 2 * longestTimeUntilTarget;
        List<ICameraController> controllers = new List<ICameraController>();
        controllers.Add(new RotatingCamera(MathF.PI * 0.15f, (float)random.NextDouble() * MathF.PI * 2, 0.03f, 4, hipHandles.Count * 0.3f, 0.9f));
        controllers.Add(new RotatingCamera(MathF.PI * 0.15f, (float)random.NextDouble() * MathF.PI * 2, 0.03f, 4, hipHandles.Count * 0.3f, 0.5f));
        for (int i = 0; i < hipHandles.Count; ++i)
        {
            controllers.Add(new FollowCamera(hipHandles[i], scene, 10, 1.5f, 2, 1));
        }
        director = new CameraDirector(controllers.ToArray(), random);
    }


    public bool Update(Scene scene, Random random, Camera camera, float accumulatedTime, float accumulatedRealTime, bool controlCamera)
    {
        if (controlCamera)
            director.Update(scene, camera, random, accumulatedTime, accumulatedRealTime);
        return accumulatedTime < targetTime;
    }
}

