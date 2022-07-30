using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;
using static AbominationInterop.SimpleSelfContainedDemo;

namespace AbominationInterop;

public enum SIMDWidth
{
    SIMD128 = 0,
    SIMD256 = 1
}

internal static class Entrypoints
{
    static InstanceDirectory<BufferPool> bufferPools;
    static InstanceDirectory<Simulation> simulations;
    static InstanceDirectory<ThreadDispatcher> threadDispatchers;

    public const string FunctionNamePrefix = "Bepu";

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Initialize))]
    public static void Initialize()
    {
        bufferPools = new InstanceDirectory<BufferPool>(0);
        simulations = new InstanceDirectory<Simulation>(1);
        threadDispatchers = new InstanceDirectory<ThreadDispatcher>(2);
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetSIMDWidth))]
    public static SIMDWidth GetSIMDWidth()
    {
        //This will have to change later, but it technically works now.
        return Vector<float>.Count == 8 ? SIMDWidth.SIMD256 : SIMDWidth.SIMD128;
    }


    /// <summary>
    /// Creates a new buffer pool.
    /// </summary>
    /// <param name="minimumBlockAllocationSize">Minimum size of individual block allocations. Must be a power of 2.
    /// Pools with single allocations larger than the minimum will use the minimum value necessary to hold one element.
    /// Buffers will be suballocated from blocks.</param>
    /// <param name="expectedUsedSlotCountPerPool">Number of suballocations to preallocate reference space for.
    /// This does not preallocate actual blocks, just the space to hold references that are waiting in the pool.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateBufferPool))]
    public static InstanceHandle CreateBufferPool(int minimumBlockAllocationSize, int expectedUsedSlotCountPerPool)
    {
        return bufferPools.Add(new BufferPool(minimumBlockAllocationSize, expectedUsedSlotCountPerPool));
    }

    /// <summary>
    /// Releases all allocations held by the buffer pool. The buffer pool remains in a usable state.
    /// </summary>
    /// <param name="handle">Buffer pool to clear.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ClearBufferPool))]
    public static void ClearBufferPool(InstanceHandle handle)
    {
        bufferPools[handle].Clear();
    }

    /// <summary>
    /// Releases all allocations held by the buffer pool and releases the buffer pool reference. The handle is invalidated.
    /// </summary>
    /// <param name="handle">Buffer pool to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyBufferPool))]
    public static void DestroyBufferPool(InstanceHandle handle)
    {
        bufferPools[handle].Clear();
        bufferPools.Remove(handle);
    }

    /// <summary>
    /// Creates a new thread dispatcher.
    /// </summary>
    /// <param name="workerCount">Number of threads to use within the thread dispatcher.</param>
    /// <param name="threadPoolAllocationBlockSize">Minimum size in bytes of blocks allocated in per-thread buffer pools. Allocations requiring more space can result in larger block sizes, but no pools will allocate smaller blocks.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateThreadDispatcher))]
    public static InstanceHandle CreateThreadDispatcher(int threadCount, int threadPoolAllocationBlockSize)
    {
        return threadDispatchers.Add(new ThreadDispatcher(threadCount, threadPoolAllocationBlockSize));
    }

    /// <summary>
    /// Releases all resources held by a thread dispatcher and invalidates its handle.
    /// </summary>
    /// <param name="handle">Thread dispatcher to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyThreadDispatcher))]
    public static void DestroyThreadDispatcher(InstanceHandle handle)
    {
        threadDispatchers[handle].Dispose();
        threadDispatchers.Remove(handle);
    }



    //We don't want to do runtime checks in the callbacks, so we jump through some fun hoops to construct a type.
    private static unsafe InstanceHandle CreateSimulationWithScalarIntegrationState<TConservationType, TSubstepUnconstrained, TIntegrateKinematicVelocities, TScalarIntegration>(
      BufferPool pool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacksInterop poseIntegratorCallbacksInterop, SolveDescription solveDescription, SimulationAllocationSizes initialAllocationSizes)
    {
        void* integrateVelocityFunction =
            poseIntegratorCallbacksInterop.UseScalarCallback != 0 ? poseIntegratorCallbacksInterop.IntegrateVelocityScalar :
            Vector<float>.Count == 4 ? poseIntegratorCallbacksInterop.IntegrateVelocitySIMD128 : poseIntegratorCallbacksInterop.IntegrateVelocitySIMD256;
        if (integrateVelocityFunction == null)
            throw new NullReferenceException("Velocity integration callback is not defined. Was the wrong callback provided for the scalar state/SIMD width?");
        var poseIntegratorCallbacks = new PoseIntegratorCallbacks<TConservationType, TSubstepUnconstrained, TIntegrateKinematicVelocities, TScalarIntegration>
        {
            InitializeFunction = poseIntegratorCallbacksInterop.Initialize,
            PrepareForIntegrationFunction = poseIntegratorCallbacksInterop.PrepareForIntegration,
            IntegrateVelocityFunction = integrateVelocityFunction
        };
        //For now, the native side can't define custom timesteppers. This isn't fundamental, but exposing it would be somewhat annoying, so punted.
        var simulation = Simulation.Create(pool, narrowPhaseCallbacks, poseIntegratorCallbacks, solveDescription, initialAllocationSizes: initialAllocationSizes);
        var handle = simulations.Add(simulation);
        //The usual narrow phase callbacks initialization could not be done because there was no handle available for the native side to use, so call it now.
        ((NarrowPhase<NarrowPhaseCallbacks>)simulation.NarrowPhase).Callbacks.Initialize(handle);
        //Same for pose integrator callbacks.
        ((PoseIntegrator<PoseIntegratorCallbacks<TConservationType, TSubstepUnconstrained, TIntegrateKinematicVelocities, TScalarIntegration>>)simulation.PoseIntegrator).Callbacks.Initialize(handle);
        return handle;
    }
    private static InstanceHandle CreateSimulationWithKinematicIntegrationState<TConservationType, TSubstepUnconstrained, TIntegrateKinematicVelocities>(
        BufferPool pool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacksInterop poseIntegratorCallbacksInterop, SolveDescription solveDescription, SimulationAllocationSizes initialAllocationSizes)
    {
        if (poseIntegratorCallbacksInterop.UseScalarCallback != 0)
            return CreateSimulationWithScalarIntegrationState<TConservationType, TSubstepUnconstrained, TIntegrateKinematicVelocities, True>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
        return CreateSimulationWithScalarIntegrationState<TConservationType, TSubstepUnconstrained, TIntegrateKinematicVelocities, False>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
    }
    private static InstanceHandle CreateSimulationWithUnconstrainedSubstepState<TConservationType, TSubstepUnconstrained>(
        BufferPool pool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacksInterop poseIntegratorCallbacksInterop, SolveDescription solveDescription, SimulationAllocationSizes initialAllocationSizes)
    {
        if (poseIntegratorCallbacksInterop.IntegrateVelocityForKinematics != 0)
            return CreateSimulationWithKinematicIntegrationState<TConservationType, TSubstepUnconstrained, True>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
        return CreateSimulationWithKinematicIntegrationState<TConservationType, TSubstepUnconstrained, False>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
    }

    private static InstanceHandle CreateSimulationWithConservationType<TConservationType>(
        BufferPool pool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacksInterop poseIntegratorCallbacksInterop, SolveDescription solveDescription, SimulationAllocationSizes initialAllocationSizes)
    {
        if (poseIntegratorCallbacksInterop.AllowSubstepsForUnconstrainedBodies != 0)
            return CreateSimulationWithUnconstrainedSubstepState<TConservationType, True>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
        return CreateSimulationWithUnconstrainedSubstepState<TConservationType, False>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);

    }
    private static InstanceHandle CreateSimulation(BufferPool pool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacksInterop poseIntegratorCallbacksInterop, SolveDescription solveDescription, SimulationAllocationSizes initialAllocationSizes)
    {
        switch (poseIntegratorCallbacksInterop.AngularIntegrationMode)
        {
            case AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque:
                return CreateSimulationWithConservationType<AngularIntegrationModeConserveWithGyroTorque>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
            case AngularIntegrationMode.ConserveMomentum:
                return CreateSimulationWithConservationType<AngularIntegrationModeConserve>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
            default:
                return CreateSimulationWithConservationType<AngularIntegrationModeNonconserving>(pool, narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
        }
    }


    /// <summary>
    /// Creates a new simulation.
    /// </summary>
    /// <param name="bufferPool">Buffer pool for the simulation's main allocations.</param>
    /// <param name="narrowPhaseCallbacksInterop">Narrow phase callbacks to be invoked by the simulation.</param>
    /// <param name="poseIntegratorCallbacksInterop">Pose integration state and callbacks to be invoked by the simulation.</param>
    /// <param name="solveDescriptionInterop">Defines velocity iteration count and substep counts for the simulation's solver.</param>
    /// <param name="initialAllocationSizes">Initial capacities to allocate within the simulation.</param>
    /// <returns></returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateSimulation))]
    public unsafe static InstanceHandle CreateSimulation(InstanceHandle bufferPool, NarrowPhaseCallbacksInterop narrowPhaseCallbacksInterop, PoseIntegratorCallbacksInterop poseIntegratorCallbacksInterop,
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
        return CreateSimulation(bufferPools[bufferPool], narrowPhaseCallbacks, poseIntegratorCallbacksInterop, solveDescription, initialAllocationSizes);
    }

    /// <summary>
    /// Destroys a simulation and invalidates its handle.
    /// </summary>
    /// <param name="handle">Simulation to destroy.</param>

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroySimulation))]
    public static unsafe void DestroySimulation(InstanceHandle handle)
    {
        simulations[handle].Dispose();
        simulations.Remove(handle);
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddBody))]
    public unsafe static BodyHandle AddBody(InstanceHandle simulationHandle, BodyDescription bodyDescription)
    {
        return simulations[simulationHandle].Bodies.Add(bodyDescription);
    }
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(RemoveBody))]
    public unsafe static void RemoveBody(InstanceHandle simulationHandle, BodyHandle bodyHandle)
    {
        simulations[simulationHandle].Bodies.Remove(bodyHandle);
    }
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddStatic))]
    public unsafe static StaticHandle AddStatic(InstanceHandle simulationHandle, StaticDescription staticDescription)
    {
        return simulations[simulationHandle].Statics.Add(staticDescription);
    }
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(RemoveStatic))]
    public unsafe static void RemoveStatic(InstanceHandle simulationHandle, StaticHandle staticHandle)
    {
        simulations[simulationHandle].Statics.Remove(staticHandle);
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = "Goingtr")]
    public static void Greetings()
    {
        Console.WriteLine("Proing");
        SimpleSelfContainedDemo.Run();
        Console.WriteLine("Proinged");
    }
}
