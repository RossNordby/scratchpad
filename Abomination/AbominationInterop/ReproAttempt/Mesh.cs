using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;


/// <summary>
/// Shape designed to contain a whole bunch of triangles. Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound clockwise in right handed coordinates or counterclockwise in left handed coordinates will generate contacts.
/// </summary>
public struct Mesh : IHomogeneousCompoundShape<Triangle, TriangleWide>
{
    /// <summary>
    /// Acceleration structure of the mesh.
    /// </summary>
    public Tree Tree;
    /// <summary>
    /// Buffer of triangles composing the mesh. Triangles will only collide with tests which see the triangle as wound clockwise in right handed coordinates or counterclockwise in left handed coordinates.
    /// </summary>
    public Buffer<Triangle> Triangles;
    internal Vector3 scale;
    internal Vector3 inverseScale;
    /// <summary>
    /// Gets or sets the scale of the mesh.
    /// </summary>
    public Vector3 Scale
    {
        get
        {
            return scale;
        }
        set
        {
            scale = value;
            inverseScale = new Vector3(
                value.X != 0 ? 1f / value.X : float.MaxValue,
                value.Y != 0 ? 1f / value.Y : float.MaxValue,
                value.Z != 0 ? 1f / value.Z : float.MaxValue);
        }
    }



    /// <summary>
    /// Creates a mesh shape.
    /// </summary>
    /// <param name="triangles">Triangles to use in the mesh.</param>
    /// <param name="scale">Scale to apply to all vertices at runtime.
    /// Note that the scale is not baked into the triangles or acceleration structure; the same set of triangles and acceleration structure can be used across multiple Mesh instances with different scales.</param>
    /// <param name="pool">Pool used to allocate acceleration structures.</param>
    public Mesh(Buffer<Triangle> triangles, in Vector3 scale, BufferPool pool) : this()
    {
        Triangles = triangles;
        Tree = new Tree(pool, triangles.Length);
        pool.Take<BoundingBox>(triangles.Length, out var boundingBoxes);
        for (int i = 0; i < triangles.Length; ++i)
        {
            ref var t = ref triangles[i];
            ref var bounds = ref boundingBoxes[i];
            bounds.Min = Vector3.Min(t.A, Vector3.Min(t.B, t.C));
            bounds.Max = Vector3.Max(t.A, Vector3.Max(t.B, t.C));
        }
        Tree.SweepBuild(pool, boundingBoxes);
        pool.Return(ref boundingBoxes);
        Scale = scale;
    }

    /// <summary>
    /// Loads a mesh from data stored in a byte buffer previously stored by the Serialize function.
    /// </summary>
    /// <param name="data">Data to load the mesh from.</param>
    /// <param name="pool">Pool to create the mesh with.</param>
    public unsafe Mesh(Span<byte> data, BufferPool pool)
    {
        if (data.Length < 16)
            throw new ArgumentException("Data is not large enough to contain a header.");
        this = default;
        Scale = Unsafe.As<byte, Vector3>(ref data[0]);
        var triangleCount = Unsafe.As<byte, int>(ref data[12]);
        var triangleByteCount = triangleCount * sizeof(Triangle);
        if (data.Length < 4 + triangleCount * sizeof(Triangle))
            throw new ArgumentException($"Data is not large enough to contain the number of triangles specified in the header, {triangleCount}.");
        Tree = new Tree();
        pool.Take(triangleCount, out Triangles);
        Unsafe.CopyBlockUnaligned(ref *(byte*)Triangles.Memory, ref data[16], (uint)triangleByteCount);
    }

    public readonly int ChildCount => Triangles.Length;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly unsafe void GetLocalChild(int triangleIndex, out Triangle target)
    {
        ref var source = ref Triangles[triangleIndex];
        target.A = scale * source.A;
        target.B = scale * source.B;
        target.C = scale * source.C;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly unsafe void GetPosedLocalChild(int triangleIndex, out Triangle target, out RigidPose childPose)
    {
        GetLocalChild(triangleIndex, out target);
        childPose = (target.A + target.B + target.C) * (1f / 3f);
        target.A -= childPose.Position;
        target.B -= childPose.Position;
        target.C -= childPose.Position;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly unsafe void GetLocalChild(int triangleIndex, ref TriangleWide target)
    {
        //This inserts a triangle into the first slot of the given wide instance.
        ref var source = ref Triangles[triangleIndex];
        Vector3Wide.WriteFirst(source.A * scale, ref target.A);
        Vector3Wide.WriteFirst(source.B * scale, ref target.B);
        Vector3Wide.WriteFirst(source.C * scale, ref target.C);
    }

    public readonly void ComputeBounds(in Quaternion orientation, out Vector3 min, out Vector3 max)
    {
        Matrix3x3.CreateFromQuaternion(orientation, out var r);
        min = new Vector3(float.MaxValue);
        max = new Vector3(-float.MaxValue);
        for (int i = 0; i < Triangles.Length; ++i)
        {
            //This isn't an ideal bounding box calculation for a mesh. 
            //-You might be able to get a win out of widely vectorizing.
            //-Indexed smooth meshes would tend to have a third as many max/min operations.
            //-Even better would be a set of extreme points that are known to fully enclose the mesh, eliminating the need to test the vast majority.
            //But optimizing this only makes sense if dynamic meshes are common, and they really, really, really should not be.
            ref var triangle = ref Triangles[i];
            Matrix3x3.Transform(scale * triangle.A, r, out var a);
            Matrix3x3.Transform(scale * triangle.B, r, out var b);
            Matrix3x3.Transform(scale * triangle.C, r, out var c);
            var min0 = Vector3.Min(a, b);
            var min1 = Vector3.Min(c, min);
            var max0 = Vector3.Max(a, b);
            var max1 = Vector3.Max(c, max);
            min = Vector3.Min(min0, min1);
            max = Vector3.Max(max0, max1);
        }
    }

    public readonly ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
    {
        return new HomogeneousCompoundShapeBatch<Mesh, Triangle, TriangleWide>(pool, initialCapacity);
    }

    unsafe struct HitLeafTester<T> : IRayLeafTester where T : IShapeRayHitHandler
    {
        public Triangle* Triangles;
        public T HitHandler;
        public Matrix3x3 Orientation;
        public Vector3 InverseScale;
        public RayData OriginalRay;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void TestLeaf(int leafIndex, RayData* rayData, float* maximumT)
        {
            ref var triangle = ref Triangles[leafIndex];
            if (Triangle.RayTest(triangle.A, triangle.B, triangle.C, rayData->Origin, rayData->Direction, out var t, out var normal) && t <= *maximumT)
            {
                //Pull the hit back into world space before handing it off to the user. This does cost a bit more, but not much, and you can always add a specialized no-transform path later.
                Matrix3x3.Transform(normal * InverseScale, Orientation, out normal);
                normal = Vector3.Normalize(normal);
                HitHandler.OnRayHit(OriginalRay, ref *maximumT, t, normal, leafIndex);
            }
        }
    }

    /// <summary>
    /// Casts a ray against the mesh. Executes a callback for every test candidate and every hit.
    /// </summary>
    /// <typeparam name="TRayHitHandler">Type of the callback to execute for every test candidate and hit.</typeparam>
    /// <param name="pose">Pose of the mesh during the ray test.</param>
    /// <param name="ray">Ray to test against the mesh.</param>
    /// <param name="maximumT">Maximum length of the ray in units of the ray direction length.</param>
    /// <param name="hitHandler">Callback to execute for every hit.</param>
    public readonly unsafe void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
    {
        HitLeafTester<TRayHitHandler> leafTester;
        leafTester.Triangles = Triangles.Memory;
        leafTester.HitHandler = hitHandler;
        Matrix3x3.CreateFromQuaternion(pose.Orientation, out leafTester.Orientation);
        leafTester.InverseScale = inverseScale;
        leafTester.OriginalRay = ray;
        Matrix3x3.TransformTranspose(ray.Origin - pose.Position, leafTester.Orientation, out var localOrigin);
        Matrix3x3.TransformTranspose(ray.Direction, leafTester.Orientation, out var localDirection);
        localOrigin *= inverseScale;
        localDirection *= inverseScale;
        Tree.RayCast(localOrigin, localDirection, ref maximumT, ref leafTester);
        //The leaf tester could have mutated the hit handler; copy it back over.
        hitHandler = leafTester.HitHandler;
    }


    /// <summary>
    /// Returns the mesh's resources to a buffer pool.
    /// </summary>
    /// <param name="bufferPool">Pool to return the mesh's resources to.</param>
    public void Dispose(BufferPool bufferPool)
    {
        bufferPool.Return(ref Triangles);
    }



    /// <summary>
    /// Type id of mesh shapes.
    /// </summary>
    public const int Id = 8;
    public readonly int TypeId => Id;

}