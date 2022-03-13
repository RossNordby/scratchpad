using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

/// <summary>
/// Defines a type capable of filtering ray test candidates and handling ray hit results.
/// </summary>
public interface IRayHitHandler
{
    void OnRayHit();
}

public interface IBroadPhaseRayTester
{
    unsafe void RayTest(RayData* rayData, float* maximumT);
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
    public Shapes Shapes;
    public ShapeRayHitHandler<TRayHitHandler> ShapeHitHandler;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe void RayTest(RayData* rayData, float* maximumT)
    {
        Shapes[0].RayTest(ref ShapeHitHandler);
    }
}
