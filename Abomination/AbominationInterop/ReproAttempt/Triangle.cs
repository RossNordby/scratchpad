using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
/// <summary>
/// Collision shape representing an individual triangle. Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound clockwise in right handed coordinates or counterclockwise in left handed coordinates will generate contacts.
/// </summary>
public struct Triangle : IConvexShape
{
    /// <summary>
    /// First vertex of the triangle in local space.
    /// </summary>
    public Vector3 A;
    /// <summary>
    /// Second vertex of the triangle in local space.
    /// </summary>
    public Vector3 B;
    /// <summary>
    /// Third vertex of the triangle in local space.
    /// </summary>
    public Vector3 C;

    /// <summary>
    /// Creates a triangle shape.
    /// </summary>
    /// <param name="a">First vertex of the triangle in local space.</param>
    /// <param name="b">Second vertex of the triangle in local space.</param>
    /// <param name="c">Third vertex of the triangle in local space.</param>
    public Triangle(in Vector3 a, in Vector3 b, in Vector3 c)
    {
        A = a;
        B = b;
        C = c;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void ComputeBounds(in Quaternion orientation, out Vector3 min, out Vector3 max)
    {
        Matrix3x3.CreateFromQuaternion(orientation, out var basis);
        Matrix3x3.Transform(A, basis, out var worldA);
        Matrix3x3.Transform(B, basis, out var worldB);
        Matrix3x3.Transform(C, basis, out var worldC);
        min = Vector3.Min(worldA, Vector3.Min(worldB, worldC));
        max = Vector3.Max(worldA, Vector3.Max(worldB, worldC));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
    {
        maximumRadius = (float)Math.Sqrt(MathHelper.Max(A.LengthSquared(), MathHelper.Max(B.LengthSquared(), C.LengthSquared())));
        maximumAngularExpansion = maximumRadius;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RayTest(in Vector3 a, in Vector3 b, in Vector3 c, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
    {
        //Note that this assumes clockwise-in-right-hand winding. Rays coming from the opposite direction pass through; triangles are one sided.
        var ab = b - a;
        var ac = c - a;
        normal = Vector3.Cross(ac, ab);
        var dn = -Vector3.Dot(direction, normal);
        if (dn <= 0)
        {
            t = 0;
            normal = new Vector3();
            return false;
        }
        var ao = origin - a;
        t = Vector3.Dot(ao, normal);
        if (t < 0)
        {
            //Impact occurred before the start of the ray.
            return false;
        }
        var aoxd = Vector3.Cross(ao, direction);
        var v = -Vector3.Dot(ac, aoxd);
        if (v < 0 || v > dn)
        {
            //Invalid barycentric coordinate for b.
            return false;
        }
        var w = Vector3.Dot(ab, aoxd);
        if (w < 0 || v + w > dn)
        {
            //Invalid barycentric coordinate for b and/or c.
            return false;
        }
        t /= dn;
        normal /= (float)Math.Sqrt(Vector3.Dot(normal, normal));
        return true;
    }

    public readonly bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
    {
        var offset = origin - pose.Position;
        Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
        Matrix3x3.TransformTranspose(offset, orientation, out var localOffset);
        Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);
        if (RayTest(A, B, C, localOffset, localDirection, out t, out normal))
        {
            Matrix3x3.Transform(normal, orientation, out normal);
            return true;
        }
        return false;
    }

    public readonly ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
    {
        return null;// new ConvexShapeBatch<Triangle, TriangleWide>(pool, initialCapacity);
    }

    /// <summary>
    /// Type id of triangle shapes.
    /// </summary>
    public const int Id = 3;
    public readonly int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
}
