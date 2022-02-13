using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;


namespace HeadlessTests23.StreamerStyle.Actions;

public class NewtZoom : IAction
{
    float targetTime;

    public void Initialize(Random random, Scene scene)
    {
        var newtCount = random.Next(1, 6);

        var span = scene.RegionOfInterest.Max - scene.RegionOfInterest.Min; 
        Mesh templateMesh;
        using (var stream = File.Open(@"Content\newt.obj", FileMode.Open))
        {
            templateMesh = MeshLoader.LoadMesh(stream, Vector3.One, scene.BufferPool);
        }

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
    }


    public bool Update(Scene scene, Random random, float accumulatedTime)
    {
        return accumulatedTime < targetTime;
    }
}

