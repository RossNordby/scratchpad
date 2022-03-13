using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

public abstract class ShapeBatch
{

    public abstract void RayTest<TRayHitHandler>(ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;
}

public abstract class ShapeBatch<TShape> : ShapeBatch where TShape : unmanaged, IShape
{
    internal Buffer<TShape> shapes;  
}

public class HomogeneousCompoundShapeBatch<TShape, TChildShape, TChildShapeWide> : ShapeBatch<TShape> where TShape : unmanaged, IHomogeneousCompoundShape<TChildShape, TChildShapeWide>
    where TChildShape : IConvexShape
    where TChildShapeWide : IShapeWide<TChildShape>
{
    public override void RayTest<TRayHitHandler>(ref TRayHitHandler hitHandler)
    {
        shapes[0].RayTest(ref hitHandler);
    }

}