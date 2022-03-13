
using BepuUtilities.Memory;
using System.Numerics;

public struct HitHandler : IShapeRayHitHandler
{
    public object Objeto;

    public void OnRayHit()
    {
        Console.WriteLine($"hit");
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
        hitHandler.Objeto = new object();
        shapes[mesh.TypeId].RayTest(ref hitHandler);

        Console.WriteLine($"Yeah! objeto: {hitHandler.Objeto}");
        pool.Clear();
    }
}

