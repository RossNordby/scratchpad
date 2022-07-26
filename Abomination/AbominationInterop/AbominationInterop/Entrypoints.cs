using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace AbominationInterop;

[StructLayout(LayoutKind.Sequential)]
public unsafe struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{

    public delegate*<InstanceHandle, void> InitializeFunction;
    public delegate*<void> DisposeFunction;
    public delegate*<int, CollidableReference, CollidableReference, float*, byte> AllowContactGenerationFunction;
    public delegate*<int, CollidablePair, int, int, byte> AllowContactGenerationBetweenChildrenFunction;
    public delegate*<int, CollidablePair, ConvexContactManifold*, PairMaterialProperties*, byte> ConfigureConvexContactManifoldFunction;
    public delegate*<int, CollidablePair, NonconvexContactManifold*, PairMaterialProperties*, byte> ConfigureNonconvexContactManifoldFunction;
    public delegate*<int, CollidablePair, int, int, ConvexContactManifold*, byte> ConfigureChildContactManifoldFunction;

    public void Initialize(Simulation simulation)
    {
        //Note that we don't use this for the interop callbacks; we don't have a simulation handle yet, so the interop can't yet refer to the simulation!
    }
    public void Initialize(InstanceHandle simulation)
    {
        if (InitializeFunction != null)
            InitializeFunction(simulation);
    }
    public void Dispose()
    {
        if (DisposeFunction != null)
            DisposeFunction();
    }

    //Note that a number of these convert refs into pointers. These are safe; all such references originate on the stack or pinned memory.
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        return AllowContactGenerationFunction(workerIndex, a, b, (float*)Unsafe.AsPointer(ref speculativeMargin)) != 0;
    }

    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return AllowContactGenerationBetweenChildrenFunction(workerIndex, pair, childIndexA, childIndexB) != 0;
    }

    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        //Can't directly expose the generic type across interop boundary, so we need two typed handlers.
        //We could use one function and pass an untyped pointer + type indicator, but that doesn't seem like a significant improvement.
        //This version can be recombined into a single template function on the other end if so desired.
        Unsafe.SkipInit(out pairMaterial);
        var pairMaterialPointer = (PairMaterialProperties*)Unsafe.AsPointer(ref pairMaterial);
        if (typeof(TManifold) == typeof(ConvexContactManifold))
        {
            return ConfigureConvexContactManifoldFunction(workerIndex, pair, (ConvexContactManifold*)Unsafe.AsPointer(ref manifold), pairMaterialPointer) != 0;
        }
        else
        {
            return ConfigureNonconvexContactManifoldFunction(workerIndex, pair, (NonconvexContactManifold*)Unsafe.AsPointer(ref manifold), pairMaterialPointer) != 0;
        }
    }

    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return ConfigureChildContactManifoldFunction(workerIndex, pair, childIndexA, childIndexB, (ConvexContactManifold*)Unsafe.AsPointer(ref manifold)) != 0;
    }
}

[StructLayout(LayoutKind.Explicit)]
public unsafe struct SolveDescriptionInterop
{
    /// <summary>
    /// Number of velocity iterations to use in the solver if there is no <see cref="VelocityIterationScheduler"/> or if it returns a non-positive value for a substep.
    /// </summary>
    [FieldOffset(0)]
    public int VelocityIterationCount;
    /// <summary>
    /// Number of substeps to execute each time the solver runs.
    /// </summary>
    [FieldOffset(4)]
    public int SubstepCount;
    /// <summary>
    /// Number of synchronzed constraint batches to use before using a fallback approach.
    /// </summary>
    [FieldOffset(8)]
    public int FallbackBatchThreshold;
    /// <summary>
    /// Callback executed to determine how many velocity iterations should be used for a given substep. If null, or if it returns a non-positive value, the <see cref="VelocityIterationCount"/> will be used instead.
    /// </summary>
    [FieldOffset(16)]
    public delegate*<int, int> VelocityIterationScheduler;
}

internal static class Entrypoints
{
    static InstanceDirectory<BufferPool> bufferPools;
    static InstanceDirectory<Simulation> simulations;

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = nameof(Initialize))]
    public static void Initialize()
    {
        bufferPools = new InstanceDirectory<BufferPool>(0);
        simulations = new InstanceDirectory<Simulation>(1);
    }

    /// <summary>
    /// Creates a new buffer pool.
    /// </summary>
    /// <param name="minimumBlockAllocationSize">Minimum size of individual block allocations. Must be a power of 2.
    /// Pools with single allocations larger than the minimum will use the minimum value necessary to hold one element.
    /// Buffers will be suballocated from blocks.</param>
    /// <param name="expectedUsedSlotCountPerPool">Number of suballocations to preallocate reference space for.
    /// This does not preallocate actual blocks, just the space to hold references that are waiting in the pool.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = nameof(CreateBufferPool))]
    public static InstanceHandle CreateBufferPool(int minimumBlockAllocationSize, int expectedUsedSlotCountPerPool)
    {
        return bufferPools.Add(new BufferPool(minimumBlockAllocationSize, expectedUsedSlotCountPerPool));
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = nameof(CreateSimulation))]
    public unsafe static InstanceHandle CreateSimulation(InstanceHandle bufferPool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacks poseIntegratorCallbacks, 
        SolveDescriptionInterop solveDescriptionInterop, Timestepper)
    {
        var solveDescription = new SolveDescription
        {
            VelocityIterationCount = solveDescriptionInterop.VelocityIterationCount,
            SubstepCount = solveDescriptionInterop.SubstepCount,
            FallbackBatchThreshold = solveDescriptionInterop.FallbackBatchThreshold,
            VelocityIterationScheduler = solveDescriptionInterop.VelocityIterationScheduler != null ? Marshal.GetDelegateForFunctionPointer<SubstepVelocityIterationScheduler>((IntPtr)solveDescriptionInterop.VelocityIterationScheduler) : null
        };
        var simulation = Simulation.Create(bufferPools[bufferPool], narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription, timestepper, initialAllocationSizes);
        simulations.Add(simulation);
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = "Goingtr")]
    public static void Greetings()
    {
        Console.WriteLine("Proing");
        SimpleSelfContainedDemo.Run();
        Console.WriteLine("Proinged");
    }
}
