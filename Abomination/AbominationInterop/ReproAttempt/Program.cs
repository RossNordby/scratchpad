
using BepuUtilities.Memory;
using System.Numerics;

public struct HitHandler : IShapeRayHitHandler
{
    public bool AllowTest(int childIndex)
    {
        Console.WriteLine($"testing: {childIndex}");
        return true;
    }

    public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, int childIndex)
    {
        Console.WriteLine($"hit: {t}");
    }
}

public static class Program
{
    public static void Main()
    {

        Console.WriteLine("Hello, World!");

        BufferPool pool = new BufferPool();
        Shapes shapes = new Shapes(pool, 32);
        //This is a bit poopy. that's ok.
        pool.Take<Triangle>(4, out var trianglesBuffer);
        for (int i = 0; i < trianglesBuffer.Length; ++i)
        {
            trianglesBuffer[i] = new Triangle(new Vector3(0, i, 0), new Vector3(1, i, 0), new Vector3(0, i, 1));

        }
        Mesh mesh = new Mesh(trianglesBuffer, Vector3.One, pool);
        shapes.Add(mesh);

        float hmm = 0;
        HitHandler hitHandler = default;
        shapes[mesh.TypeId].RayTest(0, RigidPose.Identity, new RayData { Direction = new Vector3(1, 1, 1), Id = 0, Origin = default }, ref hmm, ref hitHandler);

        Console.WriteLine("Yeah!");
        pool.Clear();
    }
}

