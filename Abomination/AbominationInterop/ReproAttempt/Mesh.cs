using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;


/// <summary>
/// Shape designed to contain a whole bunch of triangles. Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound clockwise in right handed coordinates or counterclockwise in left handed coordinates will generate contacts.
/// </summary>
public struct Mesh : IHomogeneousCompoundShape<Triangle, TriangleWide>
{

    public readonly ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
    {
        return new HomogeneousCompoundShapeBatch<Mesh, Triangle, TriangleWide>();
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


}