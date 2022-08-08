
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AbominationInterop;

public static partial class Entrypoints
{
    /// <summary>
    /// Adds a sphere shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="sphere">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddSphere))]
    public unsafe static TypedIndex AddSphere([TypeName(SimulationName)] InstanceHandle simulationHandle, Sphere sphere)
    {
        return simulations[simulationHandle].Shapes.Add(sphere);
    }

    /// <summary>
    /// Adds a capsule shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="capsule">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddCapsule))]
    public unsafe static TypedIndex AddCapsule([TypeName(SimulationName)] InstanceHandle simulationHandle, Capsule capsule)
    {
        return simulations[simulationHandle].Shapes.Add(capsule);
    }

    /// <summary>
    /// Adds a box shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="box">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddBox))]
    public unsafe static TypedIndex AddBox([TypeName(SimulationName)] InstanceHandle simulationHandle, Box box)
    {
        return simulations[simulationHandle].Shapes.Add(box);
    }

    /// <summary>
    /// Adds a triangle shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="triangle">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddTriangle))]
    public unsafe static TypedIndex AddTriangle([TypeName(SimulationName)] InstanceHandle simulationHandle, Triangle triangle)
    {
        return simulations[simulationHandle].Shapes.Add(triangle);
    }

    /// <summary>
    /// Adds a cylinder shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="cylinder">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddCylinder))]
    public unsafe static TypedIndex AddCylinder([TypeName(SimulationName)] InstanceHandle simulationHandle, Cylinder cylinder)
    {
        return simulations[simulationHandle].Shapes.Add(cylinder);
    }

    /// <summary>
    /// Adds a convex hull shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="convexHull">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddConvexHull))]
    public unsafe static TypedIndex AddConvexHull([TypeName(SimulationName)] InstanceHandle simulationHandle, ConvexHull convexHull)
    {
        return simulations[simulationHandle].Shapes.Add(convexHull);
    }

    /// <summary>
    /// Adds a compound shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="bigCompound">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddCompound))]
    public unsafe static TypedIndex AddCompound([TypeName(SimulationName)] InstanceHandle simulationHandle, Compound bigCompound)
    {
        return simulations[simulationHandle].Shapes.Add(bigCompound);
    }

    /// <summary>
    /// Adds a big compound shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="bigCompound">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddBigCompound))]
    public unsafe static TypedIndex AddBigCompound([TypeName(SimulationName)] InstanceHandle simulationHandle, BigCompound bigCompound)
    {
        return simulations[simulationHandle].Shapes.Add(bigCompound);
    }

    /// <summary>
    /// Adds a mesh shape to the simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
    /// <param name="mesh">Shape to add to the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddMesh))]
    public unsafe static TypedIndex AddMesh([TypeName(SimulationName)] InstanceHandle simulationHandle, Mesh mesh)
    {
        return simulations[simulationHandle].Shapes.Add(mesh);
    }

    /// <summary>
    /// Removes a shape from the simulation. Does not return any shape allocated buffers to buffer pools.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
    /// <param name="shape">Shape to remove from the simulation.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(RemoveShape))]
    public unsafe static void RemoveShape([TypeName(SimulationName)] InstanceHandle simulationHandle, TypedIndex shape)
    {
        simulations[simulationHandle].Shapes.Remove(shape);
    }

    /// <summary>
    /// Removes a shape from the simulation. If the shape has resources that were allocated from a buffer pool, they will be returned to the specified pool.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
    /// <param name="bufferPoolHandle">Buffer pool to return shape resources to, if any.</param>
    /// <param name="shape">Shape to remove from the simulation.</param>
    /// <remarks>The same buffer pool must be used for both allocation and deallocation.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(RemoveAndDestroyShape))]
    public unsafe static void RemoveAndDestroyShape([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, TypedIndex shape)
    {
        simulations[simulationHandle].Shapes.RemoveAndDispose(shape, bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Removes a shape and all references child shapes from the simulation. If the shapes had resources that were allocated from a buffer pool, they will be returned to the specified pool.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
    /// <param name="bufferPoolHandle">Buffer pool to return shape resources to, if any.</param>
    /// <param name="shape">Shape to remove from the simulation.</param>
    /// <remarks>The same buffer pool must be used for both allocation and deallocation.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(RemoveAndDestroyShapeRecursively))]
    public unsafe static void RemoveAndDestroyShapeRecursively([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, TypedIndex shape)
    {
        simulations[simulationHandle].Shapes.RecursivelyRemoveAndDispose(shape, bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Creates a convex hull shape from a point set.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to allocate resources from for the compound's acceleration structures.</param>
    /// <param name="points">Points in the convex hull.</param>
    /// <param name="centerOfMass">Center of mass computed for the hull and subtracted from all the points in the points used for the final shape.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateBigCompound))]
    public unsafe static ConvexHull CreateConvexHull([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, [TypeName("Buffer<CompoundChild>")] Buffer<Vector3> points, Vector3* centerOfMass)
    {
        ConvexHullHelper.CreateShape(points, bufferPools[bufferPoolHandle], out *centerOfMass, out var hull);
        return hull;
    }

    /// <summary>
    /// Returns buffers allocated for a convex hull shape.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
    /// <param name="convexHull">Convex hull to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyConvexHull))]
    public unsafe static void DestroyConvexHull([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, ConvexHull* convexHull)
    {
        convexHull->Dispose(bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Returns buffers allocated for a compound shape.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
    /// <param name="compound">Compound to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyCompound))]
    public unsafe static void DestroyCompound([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, Compound* compound)
    {
        compound->Dispose(bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Creates a big compound shape from a list of children.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to which the shapes referenced by the compound children belong.</param>
    /// <param name="bufferPoolHandle">Buffer pool to allocate resources from for the compound's acceleration structures.</param>
    /// <param name="children">Children of the compound.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateBigCompound))]
    public unsafe static BigCompound CreateBigCompound([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, [TypeName("Buffer<CompoundChild>")] Buffer<CompoundChild> children)
    {
        return new BigCompound(children, simulations[simulationHandle].Shapes, bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Returns buffers allocated for a big compound shape.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
    /// <param name="bigCompound">Big compound to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyBigCompound))]
    public unsafe static void DestroyBigCompound([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, BigCompound* bigCompound)
    {
        bigCompound->Dispose(bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Creates a mesh shape from triangles.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to allocate resources from for the compound's acceleration structures.</param>
    /// <param name="triangles">Triangles composing the mesh.</param>
    /// <param name="scale">Scale of the mesh.</param>
    /// <remarks>This uses a pretty old sweep builder. Large meshes will take a while. There are ways to do this much faster if required; see https://github.com/bepu/bepuphysics2/blob/master/Demos/DemoMeshHelper.cs#L186.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateMesh))]
    public unsafe static Mesh CreateMesh([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, [TypeName("Buffer<Triangle>")] Buffer<Triangle> triangles, Vector3 scale)
    {
        return new Mesh(triangles, scale, bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Returns buffers allocated for a mesh shape.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
    /// <param name="mesh">Mesh to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyMesh))]
    public unsafe static void DestroyMesh([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, Mesh* mesh)
    {
        mesh->Dispose(bufferPools[bufferPoolHandle]);
    }

    /// <summary>
    /// Computes the inertia of a sphere.
    /// </summary>
    /// <param name="sphere">Shape to compute the inertia of.</param>
    /// <param name="mass">Mass to use in the inertia calculation.</param>
    /// <returns>Inertia of the shape.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeSphereInertia))]
    public unsafe static BodyInertia ComputeSphereInertia(Sphere sphere, float mass)
    {
        return sphere.ComputeInertia(mass);
    }

    /// <summary>
    /// Computes the inertia of a capsule.
    /// </summary>
    /// <param name="capsule">Shape to compute the inertia of.</param>
    /// <param name="mass">Mass to use in the inertia calculation.</param>
    /// <returns>Inertia of the shape.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeCapsuleInertia))]
    public unsafe static BodyInertia ComputeCapsuleInertia(Capsule capsule, float mass)
    {
        return capsule.ComputeInertia(mass);
    }

    /// <summary>
    /// Computes the inertia of a box.
    /// </summary>
    /// <param name="box">Shape to compute the inertia of.</param>
    /// <param name="mass">Mass to use in the inertia calculation.</param>
    /// <returns>Inertia of the shape.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeBoxInertia))]
    public unsafe static BodyInertia ComputeBoxInertia(Box box, float mass)
    {
        return box.ComputeInertia(mass);
    }

    /// <summary>
    /// Computes the inertia of a triangle.
    /// </summary>
    /// <param name="triangle">Shape to compute the inertia of.</param>
    /// <param name="mass">Mass to use in the inertia calculation.</param>
    /// <returns>Inertia of the shape.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeTriangleInertia))]
    public unsafe static BodyInertia ComputeTriangleInertia(Triangle triangle, float mass)
    {
        return triangle.ComputeInertia(mass);
    }

    /// <summary>
    /// Computes the inertia of a cylinder.
    /// </summary>
    /// <param name="cylinder">Shape to compute the inertia of.</param>
    /// <param name="mass">Mass to use in the inertia calculation.</param>
    /// <returns>Inertia of the shape.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeCylinderInertia))]
    public unsafe static BodyInertia ComputeCylinderInertia(Cylinder cylinder, float mass)
    {
        return cylinder.ComputeInertia(mass);
    }

    /// <summary>
    /// Computes the inertia of a convex hull.
    /// </summary>
    /// <param name="convexHull">Shape to compute the inertia of.</param>
    /// <param name="mass">Mass to use in the inertia calculation.</param>
    /// <returns>Inertia of the shape.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeConvexHullInertia))]
    public unsafe static BodyInertia ComputeConvexHullInertia(ConvexHull convexHull, float mass)
    {
        return convexHull.ComputeInertia(mass);
    }

    /// <summary>
    /// Computes the inertia associated with a set of compound children. Does not recenter the children.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to which the shapes referenced by the compound children belong.</param>
    /// <param name="children">Children of the compound.</param>
    /// <param name="childMasses">Masses of the children composing the compound.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeCompoundInertia))]
    public unsafe static BodyInertia ComputeCompoundInertia([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName("Buffer<CompoundChild>")] Buffer<CompoundChild> children, [TypeName("Buffer<float>")] Buffer<float> childMasses)
    {
        return CompoundBuilder.ComputeInertia(children, childMasses, simulations[simulationHandle].Shapes);
    }

    /// <summary>
    /// Computes the inertia associated with a set of compound children. Recenters all children onto the computed local center of mass.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to which the shapes referenced by the compound children belong.</param>
    /// <param name="children">Children of the compound.</param>
    /// <param name="childMasses">Masses of the children composing the compound.</param>
    /// <param name="centerOfMass">Computed center of mass that was subtracted from the position of compound children.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeCompoundInertiaWithRecentering))]
    public unsafe static BodyInertia ComputeCompoundInertiaWithRecentering([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName("Buffer<CompoundChild>")] Buffer<CompoundChild> children, [TypeName("Buffer<float>")] Buffer<float> childMasses, Vector3* centerOfMass)
    {
        return CompoundBuilder.ComputeInertia(children, childMasses, simulations[simulationHandle].Shapes, out *centerOfMass);
    }

    /// <summary>
    /// Computes the inertia associated with a mesh by treating its triangles as a soup with no volume. Does not recenter the triangles on a computed center of mass.
    /// </summary>
    /// <param name="mesh">Mesh to compute the inertia of.</param>
    /// <param name="mass">Mass of the mesh.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeOpenMeshInertia))]
    public unsafe static BodyInertia ComputeOpenMeshInertia(Mesh mesh, float mass)
    {
        return mesh.ComputeOpenInertia(mass);
    }

    /// <summary>
    /// Computes the inertia associated with a mesh by treating it as a closed volume. Does not recenter the triangles on a computed center of mass.
    /// </summary>
    /// <param name="mesh">Mesh to compute the inertia of.</param>
    /// <param name="mass">Mass of the mesh.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeClosedMeshInertia))]
    public unsafe static BodyInertia ComputeClosedMeshInertia(Mesh mesh, float mass)
    {
        return mesh.ComputeClosedInertia(mass);
    }

    /// <summary>
    /// Computes the inertia associated with a mesh by treating its triangles as a soup with no volume. Recenters all children onto the computed local center of mass.
    /// </summary>
    /// <param name="mesh">Mesh to compute the inertia of.</param>
    /// <param name="mass">Mass of the mesh.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeOpenMeshInertiaWithRecentering))]
    public unsafe static BodyInertia ComputeOpenMeshInertiaWithRecentering(Mesh mesh, float mass, Vector3* centerOfMass)
    {
        return mesh.ComputeOpenInertia(mass, out *centerOfMass);
    }

    /// <summary>
    /// Computes the inertia associated with a mesh by treating it as a closed volume. Recenters all children onto the computed local center of mass.
    /// </summary>
    /// <param name="mesh">Mesh to compute the inertia of.</param>
    /// <param name="mass">Mass of the mesh.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ComputeClosedMeshInertiaWithRecentering))]
    public unsafe static BodyInertia ComputeClosedMeshInertiaWithRecentering(Mesh mesh, float mass, Vector3* centerOfMass)
    {
        return mesh.ComputeClosedInertia(mass, out *centerOfMass);
    }
}
