
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;

public interface IShapeRayHitHandler
{
    void OnRayHit();
}

public interface IHomogeneousCompoundShape
{
    void RayTest<TRayHitHandler>(ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;
}



public struct Mesh : IHomogeneousCompoundShape
{
    public readonly unsafe void RayTest<TRayHitHandler>(ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
    {

    }


}
public class HomogeneousCompoundShapeBatch<TShape> where TShape : unmanaged, IHomogeneousCompoundShape
{
    internal Buffer<TShape> shapes;
    public void RayTest<TRayHitHandler>(ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
    {
        shapes[0].RayTest(ref hitHandler);
    }

}

public interface IRayHitHandler
{
    void OnRayHit();
}

public interface IBroadPhaseRayTester
{
    unsafe void RayTest(float* maximumT);
}

struct ShapeRayHitHandler<TRayHitHandler> : IShapeRayHitHandler where TRayHitHandler : IRayHitHandler
{
    public TRayHitHandler HitHandler;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnRayHit()
    {
        HitHandler.OnRayHit();
    }
}

struct RayHitDispatcher<TRayHitHandler> : IBroadPhaseRayTester where TRayHitHandler : IRayHitHandler
{
    public HomogeneousCompoundShapeBatch<Mesh> Shapes;
    public ShapeRayHitHandler<TRayHitHandler> ShapeHitHandler;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe void RayTest(float* maximumT)
    {
        Shapes.RayTest(ref ShapeHitHandler);
    }
}


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
        //Shapes shapes = new Shapes();
        Mesh mesh = new Mesh();
        Console.WriteLine($"mesh: {mesh}");
        HitHandler hitHandler = default;
        hitHandler.Objeto = new object();
        var batch = new HomogeneousCompoundShapeBatch<Mesh>();
        batch.RayTest(ref hitHandler);
        //shapes[0].RayTest(ref hitHandler);

        Console.WriteLine($"Yeah! objeto: {hitHandler.Objeto}");
        pool.Clear();
    }
}

