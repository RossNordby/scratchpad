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
        this.scale = scale;
    }


    public readonly ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
    {
        return new HomogeneousCompoundShapeBatch<Mesh, Triangle, TriangleWide>(pool, initialCapacity);
    }


    /// <summary>
    /// Casts a ray against the mesh. Executes a callback for every test candidate and every hit.
    /// </summary>
    /// <typeparam name="TRayHitHandler">Type of the callback to execute for every test candidate and hit.</typeparam>
    /// <param name="pose">Pose of the mesh during the ray test.</param>
    /// <param name="ray">Ray to test against the mesh.</param>
    /// <param name="maximumT">Maximum length of the ray in units of the ray direction length.</param>
    /// <param name="hitHandler">Callback to execute for every hit.</param>
    public readonly unsafe void RayTest<TRayHitHandler>(ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
    {

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