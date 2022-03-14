
using System.Runtime.CompilerServices;

public interface IShapeRayHitHandler
{
    void OnRayHit();
}

public interface IHomogeneousCompoundShape
{
    void RayTest<TRayHitHandler>() where TRayHitHandler : struct, IShapeRayHitHandler;
}



public struct Mesh : IHomogeneousCompoundShape
{
    public readonly unsafe void RayTest<TRayHitHandler>() where TRayHitHandler : struct, IShapeRayHitHandler
    {

    }


}
public class HomogeneousCompoundShapeBatch<TShape> where TShape : unmanaged, IHomogeneousCompoundShape
{
    public void RayTest<TRayHitHandler>() where TRayHitHandler : struct, IShapeRayHitHandler
    {
        default(TShape).RayTest<TRayHitHandler>();
    }

}

public interface IRayHitHandler
{
    void OnRayHit();
}


struct ShapeRayHitHandler<TRayHitHandler> : IShapeRayHitHandler where TRayHitHandler : IRayHitHandler
{
    public void OnRayHit()
    {
    }
}

struct RayHitDispatcher<TRayHitHandler> where TRayHitHandler : IRayHitHandler
{
    public unsafe void RayTest()
    {
        new HomogeneousCompoundShapeBatch<Mesh>().RayTest<ShapeRayHitHandler<TRayHitHandler>>();
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

        //Shapes shapes = new Shapes();
        Mesh mesh = new Mesh();
        Console.WriteLine($"mesh: {mesh}");
        HitHandler hitHandler = default;
        hitHandler.Objeto = new object();
        var batch = new HomogeneousCompoundShapeBatch<Mesh>();
        batch.RayTest<HitHandler>();
        //shapes[0].RayTest(ref hitHandler);

        Console.WriteLine($"Yeah! objeto: {hitHandler.Objeto}");
    }
}

