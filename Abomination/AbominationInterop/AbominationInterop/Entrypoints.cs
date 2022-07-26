using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
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

    public const string FunctionNamePrefix = "Bepu";

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Initialize))]
    public static void Initialize()
    {
        bufferPools = new InstanceDirectory<BufferPool>(0);
        simulations = new InstanceDirectory<Simulation>(1);
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

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = "Goingtr")]
    public static void Greetings()
    {
        Console.WriteLine("Proing");
        SimpleSelfContainedDemo.Run();
        Console.WriteLine("Proinged");
    }
}
