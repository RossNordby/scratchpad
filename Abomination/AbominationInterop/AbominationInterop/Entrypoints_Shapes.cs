
using BepuPhysics.Collidables;
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
}
