using BepuPhysics;
using BepuUtilities;
using BepuUtilities.Collections;
using System.Numerics;

namespace HeadlessTests23.StreamerStyle.Actions;

public class Clonesmash : IAction
{
    float targetTime;
    public void Initialize(Random random, Scene scene)
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
    }


    public bool Update(Scene scene, Random random, float accumulatedTime)
    {
        return accumulatedTime < targetTime;
    }
}

