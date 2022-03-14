
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
        Shapes shapes = new Shapes();
        Mesh mesh = new Mesh();
        Console.WriteLine($"mesh: {mesh}");
        HitHandler hitHandler = default;
        hitHandler.Objeto = new object();
        shapes[0].RayTest(ref hitHandler);

        Console.WriteLine($"Yeah! objeto: {hitHandler.Objeto}");
        pool.Clear();
    }
}

