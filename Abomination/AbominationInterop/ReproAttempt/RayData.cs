using System.Numerics;
using System.Runtime.CompilerServices;

public struct RayData
{
    public Vector3 Origin;
    public int Id;
    public Vector3 Direction;
}

/// <summary>
/// Ray representation designed for quicker intersection against axis aligned bounding boxes.
/// </summary>
public struct TreeRay
{
    public Vector3 OriginOverDirection;
    public float MaximumT;
    public Vector3 InverseDirection;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void CreateFrom(in Vector3 origin, in Vector3 direction, float maximumT, out TreeRay treeRay)
    {
        //Note that this division has two odd properties:
        //1) If the local direction has a near zero component, it is clamped to a nonzero but extremely small value. This is a hack, but it works reasonably well.
        //The idea is that any interval computed using such an inverse would be enormous. Those values will not be exactly accurate, but they will never appear as a result
        //because a parallel ray will never actually intersect the surface. The resulting intervals are practical approximations of the 'true' infinite intervals.
        //2) To compensate for the clamp and abs, we reintroduce the sign in the numerator.
        //TODO: There is a small chance that a gather/scatter vectorized implementation would be a win. Pretty questionable, though.
        treeRay.InverseDirection = new Vector3(direction.X < 0 ? -1 : 1, direction.Y < 0 ? -1 : 1, direction.Z < 0 ? -1 : 1) / Vector3.Max(new Vector3(1e-15f), Vector3.Abs(direction));
        treeRay.MaximumT = maximumT;
        treeRay.OriginOverDirection = origin * treeRay.InverseDirection;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void CreateFrom(in Vector3 origin, in Vector3 direction, float maximumT, int id, out RayData rayData, out TreeRay treeRay)
    {
        rayData.Origin = origin;
        rayData.Id = id;
        rayData.Direction = direction;
        CreateFrom(origin, direction, maximumT, out treeRay);
    }
}