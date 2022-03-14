public interface IHomogeneousCompoundShape
{
    void RayTest<TRayHitHandler>() where TRayHitHandler : struct;
}

public struct Mesh : IHomogeneousCompoundShape
{
    public readonly unsafe void RayTest<TRayHitHandler>() where TRayHitHandler : struct
    {

    }
}
public class HomogeneousCompoundShapeBatch<TShape> where TShape : unmanaged, IHomogeneousCompoundShape
{
    public void RayTest<TRayHitHandler>() where TRayHitHandler : struct
    {
        default(TShape).RayTest<TRayHitHandler>();
    }

}

struct ShapeRayHitHandler<TRayHitHandler>
{
}

struct RayHitDispatcher<TRayHitHandler>
{
    public unsafe void RayTest()
    {
        new HomogeneousCompoundShapeBatch<Mesh>().RayTest<ShapeRayHitHandler<TRayHitHandler>>();
    }
}

public static class Program
{
    public static void Main()
    {
        new HomogeneousCompoundShapeBatch<Mesh>().RayTest<int>();
    }
}

