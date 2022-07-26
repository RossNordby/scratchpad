using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;
using static AbominationInterop.SimpleSelfContainedDemo;

namespace AbominationInterop;

/// <summary>
/// Vector3Wide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
public struct Vector3SIMD128
{
    public Vector128<float> X;
    public Vector128<float> Y;
    public Vector128<float> Z;
}

/// <summary>
/// Vector3Wide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
public struct Vector3SIMD256
{
    public Vector256<float> X;
    public Vector256<float> Y;
    public Vector256<float> Z;
}

/// <summary>
/// QuaternionWide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
public struct QuaternionSIMD128
{
    public Vector128<float> X;
    public Vector128<float> Y;
    public Vector128<float> Z;
}

/// <summary>
/// QuaternionWide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
public struct QuaternionSIMD256
{
    public Vector256<float> X;
    public Vector256<float> Y;
    public Vector256<float> Z;
}

/// <summary>
/// BodyInertiaWide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
public struct BodyInertiaSIMD128
{
    public Vector128<float> InverseInertiaXX;
    public Vector128<float> InverseInertiaYX;
    public Vector128<float> InverseInertiaYY;
    public Vector128<float> InverseInertiaZX;
    public Vector128<float> InverseInertiaZY;
    public Vector128<float> InverseInertiaZZ;
    public Vector128<float> InverseMass;
}

/// <summary>
/// BodyInertiaWide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
public struct BodyInertiaSIMD256
{
    public Vector256<float> InverseInertiaXX;
    public Vector256<float> InverseInertiaYX;
    public Vector256<float> InverseInertiaYY;
    public Vector256<float> InverseInertiaZX;
    public Vector256<float> InverseInertiaZY;
    public Vector256<float> InverseInertiaZZ;
    public Vector256<float> InverseMass;
}

public enum SIMDWidth
{
    SIMD128,
    SIMD256
}

[StructLayout(LayoutKind.Explicit)]
public unsafe struct PoseIntegratorCallbacksInterop
{
    [FieldOffset(0)]
    public AngularIntegrationMode AngularIntegrationMode;
    [FieldOffset(4)]
    public byte AllowSubstepsForUnconstrainedBodies;
    [FieldOffset(5)]
    public byte IntegrateVelocityForKinematics;
    [FieldOffset(6)]
    public byte UseScalarCallback;

    [FieldOffset(8)]
    public delegate* unmanaged<InstanceHandle, void> Initialize;
    [FieldOffset(16)]
    public delegate* unmanaged<InstanceHandle, float, void> PrepareForIntegration;
    //There is technically no need to expose all three of these in the interop type as separate fields; we may want to change that.
    //Right now, we're doing it just so that the signature is more explicit... but that could be better handled on the native side.
    [FieldOffset(24)]
    public delegate* unmanaged<InstanceHandle, float, void> IntegrateVelocityScalar;
    [FieldOffset(32)]
    public delegate* unmanaged<InstanceHandle, Vector128<int>, Vector3SIMD128*, QuaternionSIMD128*, BodyInertiaSIMD128*, Vector128<int>, int, Vector128<float>, void> IntegrateVelocitySIMD128;
    [FieldOffset(40)]
    public delegate* unmanaged<InstanceHandle, Vector256<int>, Vector3SIMD256*, QuaternionSIMD256*, BodyInertiaSIMD256*, Vector256<int>, int, Vector256<float>, void> IntegrateVelocitySIMD256;
}

struct AngularIntegrationModeNonconserving { }
struct AngularIntegrationModeConserve { }
struct AngularIntegrationModeConserveWithGyroTorque { }
struct True { }
struct False { }
//These value typed generic parameters will result in all the branching conditional logic in PoseIntegratorCallbacks getting elided.
public unsafe struct PoseIntegratorCallbacks<TAngularConservationMode, TUnconstrainedSubstepping, TKinematicIntegration, TScalar> : IPoseIntegratorCallbacks
{
    public AngularIntegrationMode AngularIntegrationMode
    {
        get
        {
            if (typeof(TAngularConservationMode) == typeof(AngularIntegrationModeConserveWithGyroTorque))
                return AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque;
            if (typeof(TAngularConservationMode) == typeof(AngularIntegrationModeConserve))
                return AngularIntegrationMode.ConserveMomentum;
            return AngularIntegrationMode.Nonconserving;
        }
    }

    public bool AllowSubstepsForUnconstrainedBodies => typeof(TUnconstrainedSubstepping) == typeof(True);

    public bool IntegrateVelocityForKinematics => typeof(TKinematicIntegration) == typeof(True);

    public delegate* unmanaged<InstanceHandle, void> InitializeFunction;
    public delegate* unmanaged<InstanceHandle, float, void> PrepareForIntegrationFunction;
    public void* IntegrateVelocityFunction;
    public InstanceHandle Simulation;

    public void Initialize(Simulation simulation)
    {
        //No handle exists yet so we can't expose this to the native side.
    }

    public void Initialize(InstanceHandle simulation)
    {
        Simulation = simulation;
        if (InitializeFunction != null)
           InitializeFunction(simulation);
    }

    public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
    {
        if (typeof(TScalar) == typeof(True))
        {
            //Use the scalar codepath. Adds some overhead, but understandable that some users would want it. Working with the wide representation isn't always going to be obvious.
            //TODO
            var integrateVelocity = (delegate* unmanaged<InstanceHandle, float, void>)IntegrateVelocityFunction;
        }
        else
        {
            //Wide representation.
            if (Vector<float>.Count == 4)
            {
                var integrateVelocity = (delegate* unmanaged<InstanceHandle, Vector128<int>, Vector3SIMD128*, QuaternionSIMD128*, BodyInertiaSIMD128*, Vector128<int>, int, Vector128<float>, void>)IntegrateVelocityFunction;
                //TODO
            }
            else
            {
                Debug.Assert(Vector<float>.Count == 8, "For now we're assuming that SIMD vector width is always either 128 or 256.");
                var integrateVelocity = (delegate* unmanaged<InstanceHandle, Vector256<int>, Vector3SIMD256*, QuaternionSIMD256*, BodyInertiaSIMD256*, Vector256<int>, int, Vector256<float>, void>)IntegrateVelocityFunction;
                //TODO
            }
        }
    }

    public void PrepareForIntegration(float dt)
    {
        //Really SHOULD be a prepare function provided, but it's not technically required like the velocity integration one is.
        if (PrepareForIntegrationFunction != null)
            PrepareForIntegrationFunction(Simulation, dt);
    }
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
