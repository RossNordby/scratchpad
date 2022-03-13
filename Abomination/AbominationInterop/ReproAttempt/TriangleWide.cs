using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

public struct TriangleWide : IShapeWide<Triangle>
{
    public Vector3Wide A;
    public Vector3Wide B;
    public Vector3Wide C;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Broadcast(in Triangle shape)
    {
        Vector3Wide.Broadcast(shape.A, out A);
        Vector3Wide.Broadcast(shape.B, out B);
        Vector3Wide.Broadcast(shape.C, out C);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WriteFirst(in Triangle source)
    {
        Vector3Wide.WriteFirst(source.A, ref A);
        Vector3Wide.WriteFirst(source.B, ref B);
        Vector3Wide.WriteFirst(source.C, ref C);
    }

    public bool AllowOffsetMemoryAccess => true;
    public int InternalAllocationSize => 0;
    public void Initialize(in Buffer<byte> memory) { }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void WriteSlot(int index, in Triangle source)
    {
        GatherScatter.GetOffsetInstance(ref this, index).WriteFirst(source);
    }


    /// <summary>
    /// Minimum dot product between the detected local normal and the face normal of a triangle necessary to create contacts.
    /// </summary>
    public const float BackfaceNormalDotRejectionThreshold = -1e-2f;
    /// <summary>
    /// Epsilon to apply to testing triangles for degeneracy (which will be scaled by a pair-determined epsilon scale). Degenerate triangles do not have well defined normals and should not contribute 
    /// </summary>
    public const float DegenerateTriangleEpsilon = 1e-6f;
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeTriangleEpsilonScale(in Vector<float> abLengthSquared, in Vector<float> caLengthSquared, out Vector<float> epsilonScale)
    {
        epsilonScale = Vector.SquareRoot(Vector.Max(abLengthSquared, caLengthSquared));
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeDegenerateTriangleEpsilon(in Vector<float> abLengthSquared, in Vector<float> caLengthSquared, out Vector<float> epsilonScale, out Vector<float> epsilon)
    {
        ComputeTriangleEpsilonScale(abLengthSquared, caLengthSquared, out epsilonScale);
        epsilon = new Vector<float>(DegenerateTriangleEpsilon) * epsilonScale;
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeNondegenerateTriangleMask(in Vector3Wide ab, in Vector3Wide ca, in Vector<float> triangleNormalLength, out Vector<float> epsilonScale, out Vector<int> nondegenerateMask)
    {
        Vector3Wide.LengthSquared(ab, out var abLengthSquared);
        Vector3Wide.LengthSquared(ca, out var caLengthSquared);
        ComputeDegenerateTriangleEpsilon(abLengthSquared, caLengthSquared, out epsilonScale, out var epsilon);
        nondegenerateMask = Vector.GreaterThan(triangleNormalLength, epsilon);
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeNondegenerateTriangleMask(in Vector<float> abLengthSquared, in Vector<float> caLengthSquared, in Vector<float> triangleNormalLength, out Vector<float> epsilonScale, out Vector<int> nondegenerateMask)
    {
        ComputeDegenerateTriangleEpsilon(abLengthSquared, caLengthSquared, out epsilonScale, out var epsilon);
        nondegenerateMask = Vector.GreaterThan(triangleNormalLength, epsilon);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
    {
        Matrix3x3Wide.CreateFromQuaternion(orientations, out var basis);
        Matrix3x3Wide.TransformWithoutOverlap(A, basis, out var worldA);
        Matrix3x3Wide.TransformWithoutOverlap(B, basis, out var worldB);
        Matrix3x3Wide.TransformWithoutOverlap(C, basis, out var worldC);
        min.X = Vector.Min(worldA.X, Vector.Min(worldB.X, worldC.X));
        min.Y = Vector.Min(worldA.Y, Vector.Min(worldB.Y, worldC.Y));
        min.Z = Vector.Min(worldA.Z, Vector.Min(worldB.Z, worldC.Z));
        max.X = Vector.Max(worldA.X, Vector.Max(worldB.X, worldC.X));
        max.Y = Vector.Max(worldA.Y, Vector.Max(worldB.Y, worldC.Y));
        max.Z = Vector.Max(worldA.Z, Vector.Max(worldB.Z, worldC.Z));

        Vector3Wide.LengthSquared(A, out var aLengthSquared);
        Vector3Wide.LengthSquared(B, out var bLengthSquared);
        Vector3Wide.LengthSquared(C, out var cLengthSquared);
        maximumRadius = Vector.SquareRoot(Vector.Max(aLengthSquared, Vector.Max(bLengthSquared, cLengthSquared)));
        maximumAngularExpansion = maximumRadius;
    }

    public int MinimumWideRayCount
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get
        {
            return 2;
        }
    }
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void RayTest(ref Vector3Wide a, ref Vector3Wide b, ref Vector3Wide c, ref Vector3Wide origin, ref Vector3Wide direction, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
    {
        //Note that this assumes clockwise winding. Rays coming from the opposite direction pass through; triangles are one sided.
        Vector3Wide.Subtract(b, a, out var ab);
        Vector3Wide.Subtract(c, a, out var ac);
        Vector3Wide.CrossWithoutOverlap(ac, ab, out normal);
        Vector3Wide.Dot(direction, normal, out var dn);
        dn = -dn;
        Vector3Wide.Subtract(origin, a, out var ao);
        Vector3Wide.Dot(ao, normal, out t);
        t /= dn;
        Vector3Wide.CrossWithoutOverlap(ao, direction, out var aoxd);
        Vector3Wide.Dot(ac, aoxd, out var v);
        v = -v;
        Vector3Wide.Dot(ab, aoxd, out var w);
        Vector3Wide.Normalize(normal, out normal);
        intersected = Vector.BitwiseAnd(
            Vector.BitwiseAnd(
                Vector.GreaterThan(dn, Vector<float>.Zero),
                Vector.GreaterThanOrEqual(t, Vector<float>.Zero)),
            Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(v, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(w, Vector<float>.Zero)),
                Vector.LessThanOrEqual(v + w, dn)));
    }
    //public void RayTest(ref RigidPoseWide pose, ref RayWide ray, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
    //{
    //    Vector3Wide.Subtract(ray.Origin, pose.Position, out var offset);
    //    Matrix3x3Wide.CreateFromQuaternion(pose.Orientation, out var orientation);
    //    Matrix3x3Wide.TransformByTransposedWithoutOverlap(offset, orientation, out var localOffset);
    //    Matrix3x3Wide.TransformByTransposedWithoutOverlap(ray.Direction, orientation, out var localDirection);
    //    RayTest(ref A, ref B, ref C, ref localOffset, ref localDirection, out intersected, out t, out var localNormal);
    //    Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientation, out normal);
    //}
}
