using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;

namespace HeadlessTests24.StreamerStyle.Actions;

public class Hail : IAction
{
    float targetTime;

    Vector3 shootDirection;
    float spawnDistance;
    float accumulatedHail;
    float hailPerFrame;

    const float velocityMagnitude = 200;
    public void Initialize(Random random, Scene scene)
    {
        var sceneSpan = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;
        spawnDistance = 200 + sceneSpan.Length() * 1f;

        var frequencyRand = random.NextDouble();
        var r2 = frequencyRand * frequencyRand;
        var r4 = r2 * r2;
        var r8 = r4 * r4;
        var r16 = r8 * r8;
        hailPerFrame = 0.5f + 8 * (float)(r16 * r16);

        var yaw = (float)random.NextDouble() * MathF.PI * 2;
        var pitch = (float)random.NextDouble() * MathF.PI * 0.25f;
        var sinPitch = MathF.Sin(pitch);
        shootDirection = new Vector3(MathF.Sin(yaw) * sinPitch, -MathF.Cos(pitch), MathF.Cos(yaw) * sinPitch);

        targetTime = 18 + 8 * spawnDistance / velocityMagnitude;

    }


    public bool Update(Scene scene, Random random, float accumulatedTime, float accumulatedRealTime)
    {
        accumulatedHail += hailPerFrame;
        var sceneSpan = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min;
        while (accumulatedHail >= 1)
        {
            --accumulatedHail;
            var spawnLocation = scene.RegionOfInterest.Min + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * sceneSpan - shootDirection * spawnDistance;
            var spawnVelocity = shootDirection * velocityMagnitude;

            var radiusRand = random.NextDouble();
            var r2 = radiusRand * radiusRand;
            var sphere = new Sphere(.75f + 1 * (float)(r2 * r2));
            var inertia = sphere.ComputeInertia(2 * sphere.Radius * sphere.Radius * sphere.Radius);
            scene.Simulation.Bodies.Add(BodyDescription.CreateDynamic(spawnLocation, spawnVelocity, inertia, new (scene.Simulation.Shapes.Add(sphere)), -1));

        }
        return accumulatedTime < targetTime;
    }
}

