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
    public unsafe static InstanceHandle CreateSimulation(InstanceHandle bufferPool, NarrowPhaseCallbacksInterop narrowPhaseCallbacksInterop, PoseIntegratorCallbacks poseIntegratorCallbacks,
        SolveDescriptionInterop solveDescriptionInterop, SimulationAllocationSizes initialAllocationSizes)
    {
        var solveDescription = new SolveDescription
        {
            VelocityIterationCount = solveDescriptionInterop.VelocityIterationCount,
            SubstepCount = solveDescriptionInterop.SubstepCount,
            FallbackBatchThreshold = solveDescriptionInterop.FallbackBatchThreshold,
            VelocityIterationScheduler = solveDescriptionInterop.VelocityIterationScheduler != null ? Marshal.GetDelegateForFunctionPointer<SubstepVelocityIterationScheduler>((IntPtr)solveDescriptionInterop.VelocityIterationScheduler) : null
        };
        var narrowPhaseCallbacks = new NarrowPhaseCallbacks
        {
            InitializeFunction = narrowPhaseCallbacksInterop.InitializeFunction,
            DisposeFunction = narrowPhaseCallbacksInterop.DisposeFunction,
            AllowContactGenerationFunction = narrowPhaseCallbacksInterop.AllowContactGenerationFunction,
            AllowContactGenerationBetweenChildrenFunction = narrowPhaseCallbacksInterop.AllowContactGenerationBetweenChildrenFunction,
            ConfigureConvexContactManifoldFunction = narrowPhaseCallbacksInterop.ConfigureConvexContactManifoldFunction,
            ConfigureNonconvexContactManifoldFunction = narrowPhaseCallbacksInterop.ConfigureNonconvexContactManifoldFunction,
            ConfigureChildContactManifoldFunction = narrowPhaseCallbacksInterop.ConfigureChildContactManifoldFunction
        };
        //For now, the native side can't define custom timesteppers. This isn't fundamental, but exposing it would be somewhat annoying, so punted.
        var simulation = Simulation.Create(bufferPools[bufferPool], narrowPhaseCallbacksInterop, poseIntegratorCallbacks, solveDescription, initialAllocationSizes);
        var handle = simulations.Add(simulation);
        //The usual narrow phase callbacks initialization could not be done because there was no handle available for the native side to use, so call it now.
        ((NarrowPhase<NarrowPhaseCallbacks>)simulation.NarrowPhase).Callbacks.Initialize(handle);
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = "Goingtr")]
    public static void Greetings()
    {
        Console.WriteLine("Proing");
        SimpleSelfContainedDemo.Run();
        Console.WriteLine("Proinged");
    }
}
