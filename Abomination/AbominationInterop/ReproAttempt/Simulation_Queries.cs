using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;



/// <summary>
/// Uses a bitpacked representation to refer to a body or static collidable.
/// </summary>
public struct CollidableReference
{
    /// <summary>
    /// Bitpacked representation of the collidable reference.
    /// </summary>
    public uint Packed;


    /// <summary>
    /// Gets the integer value of the handle of the owner of the collidable referred to by this instance.
    /// </summary>
    public int RawHandleValue
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            return (int)(Packed & 0x3FFFFFFF);
        }
    }

}

/// <summary>
/// Defines a type capable of filtering ray test candidates and handling ray hit results.
/// </summary>
public interface IRayHitHandler
{
    /// <summary>
    /// Checks whether a collidable identified by the acceleration structure should be tested against a ray.
    /// </summary>
    /// <param name="collidable">Candidate collidable for ray testing.</param>
    /// <returns>True if the collidable should be tested by the ray, false otherwise.</returns>
    bool AllowTest(CollidableReference collidable);
    /// <summary>
    /// Checks whether the child of a collidable should be tested against a ray. Only called by shape types that can have more than one child.
    /// </summary>
    /// <param name="collidable">Parent of the candidate.</param>
    /// <param name="childIndex">Index of the candidate in the parent collidable.</param>
    /// <returns>True if the child should be tested by the ray, false otherwise.</returns>
    bool AllowTest(CollidableReference collidable, int childIndex);
    /// <summary>
    /// Called when a ray impact has been found.
    /// </summary>
    /// <param name="ray">Information about the ray associated with this hit.</param>
    /// <param name="maximumT">Maximum distance along the ray that the traversal is allowed to go in units of ray direction length. Can be set to limit future tests.</param>
    /// <param name="t">Distance along the ray to the impact in units of ray direction length. In other words, hitLocation = ray.Origin + ray.Direction * t.</param>
    /// <param name="normal">Surface normal at the hit location.</param>
    /// <param name="collidable">Collidable hit by the ray.</param>
    /// <param name="childIndex">Index of the hit child. For convex shapes or other types that don't have multiple children, this is always zero.</param>
    void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, CollidableReference collidable, int childIndex);
}

public interface IBroadPhaseRayTester
{
    unsafe void RayTest(CollidableReference collidable, RayData* rayData, float* maximumT);
}

struct ShapeRayHitHandler<TRayHitHandler> : IShapeRayHitHandler where TRayHitHandler : IRayHitHandler
{
    public TRayHitHandler HitHandler;
    public CollidableReference Collidable;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool AllowTest(int childIndex)
    {
        return HitHandler.AllowTest(Collidable, childIndex);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnRayHit(in RayData ray, ref float maximumT, float t, in Vector3 normal, int childIndex)
    {
        HitHandler.OnRayHit(ray, ref maximumT, t, normal, Collidable, childIndex);
    }
}

struct RayHitDispatcher<TRayHitHandler> : IBroadPhaseRayTester where TRayHitHandler : IRayHitHandler
{
    public Simulation Simulation;
    public ShapeRayHitHandler<TRayHitHandler> ShapeHitHandler;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe void RayTest(CollidableReference collidable, RayData* rayData, float* maximumT)
    {
        if (ShapeHitHandler.HitHandler.AllowTest(collidable))
        {
            ShapeHitHandler.Collidable = collidable;
            Simulation.Shapes[0].RayTest(0, RigidPose.Identity, *rayData, ref *maximumT, ref ShapeHitHandler);
        }
    }
}

public class Simulation
{
    public Shapes Shapes;

    /// <summary>
    /// Intersects a ray against the simulation.
    /// </summary>
    /// <typeparam name="THitHandler">Type of the callbacks to execute on ray-object intersections.</typeparam>
    /// <param name="origin">Origin of the ray to cast.</param>
    /// <param name="direction">Direction of the ray to cast.</param>
    /// <param name="maximumT">Maximum length of the ray traversal in units of the direction's length.</param>
    /// <param name="hitHandler">callbacks to execute on ray-object intersections.</param>
    /// <param name="id">User specified id of the ray.</param>
    public unsafe void RayCast<THitHandler>(in Vector3 origin, in Vector3 direction, float maximumT, ref THitHandler hitHandler, int id = 0) where THitHandler : struct, IRayHitHandler
    {
        RayHitDispatcher<THitHandler> dispatcher;
        dispatcher.ShapeHitHandler.HitHandler = hitHandler;
        dispatcher.ShapeHitHandler.Collidable = default;
        dispatcher.Simulation = this;
        //BroadPhase.RayCast(origin, direction, maximumT, ref dispatcher, id);
        //The hit handler was copied to pass it into the child processing; since the user may (and probably does) rely on mutations, copy it back to the original reference.
        hitHandler = dispatcher.ShapeHitHandler.HitHandler;
    }
}