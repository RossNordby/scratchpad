using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Actions;

public class Shootiepatootie : IAction
{
    float targetTime;
    QuickList<BodyHandle> handles;
    public unsafe void Initialize(Random random, Scene scene)
    {
        var rand = random.NextDouble();
        var count = 5 + (int)(9 * rand * rand);
        handles = new QuickList<BodyHandle>(count, scene.BufferPool);

        var sceneSpan = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;
        var targetMin = scene.RegionOfInterest.Min + sceneSpan * 0.2f;
        var targetSpan = sceneSpan * 0.6f;
        var distance = 50 + sceneSpan.Length() * 1f;
        var velocityMagnitude = 70 + 80 * (float)random.NextDouble();

        var longestTimeUntilTarget = 0f;

        Span<float> radii = stackalloc float[count];
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


            var radiusRand = random.NextDouble();
            var r2 = radiusRand * radiusRand;
            var r4 = r2 * r2;
            var r8 = r4 * r4;
            var sphere = new Sphere(3.5f + 11 * (float)(r8 * r8 * r8 * r8));
            radii[i] = sphere.Radius;
            var inertia = sphere.ComputeInertia(4 * sphere.Radius * sphere.Radius * sphere.Radius);
            handles.AllocateUnsafely() = scene.Simulation.Bodies.Add(BodyDescription.CreateDynamic(start, new BodyVelocity(velocity), inertia, scene.Simulation.Shapes.Add(sphere), -1));
        }

        targetTime = 12 + 1.5f * longestTimeUntilTarget;
    }


    public bool Update(Scene scene, Random random, float accumulatedTime)
    {
        return accumulatedTime < targetTime;
    }
}

