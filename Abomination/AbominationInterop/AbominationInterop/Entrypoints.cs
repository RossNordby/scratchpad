﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;
using static AbominationInterop.SimpleSelfContainedDemo;

namespace AbominationInterop;

public enum SIMDWidth
{
    SIMD128 = 0,
    SIMD256 = 1,
    SIMD512 = 2
}

public class TypeNameAttribute : Attribute
{
    public string TypeName;
    public TypeNameAttribute(string typeName)
    {
        TypeName = typeName;
    }
}

public static partial class Entrypoints
{
    static InstanceDirectory<BufferPool>? bufferPools;
    static InstanceDirectory<Simulation>? simulations;
    static InstanceDirectory<ThreadDispatcher>? threadDispatchers;

    public const string FunctionNamePrefix = "";
    //These look a little odd. They're just the names of the handle types on the native side. On the C# side, they're all just InstanceHandle since we didn't want to bother doing type reinterpretation.
    public const string BufferPoolName = nameof(BufferPool) + "Handle";
    public const string SimulationName = nameof(Simulation) + "Handle";
    public const string ThreadDispatcherName = nameof(ThreadDispatcher) + "Handle";


    /// <summary>
    /// Initializes the interop structures.
    /// </summary>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Initialize))]
    public static void Initialize()
    {
        if (bufferPools != null || bufferPools != null || simulations != null)
        {
            throw new InvalidOperationException("Interop structures already exist, can't initialize again. Is a destroy missing somewhere?");
        }
        bufferPools = new InstanceDirectory<BufferPool>(0);
        simulations = new InstanceDirectory<Simulation>(1);
        threadDispatchers = new InstanceDirectory<ThreadDispatcher>(2);
    }


    /// <summary>
    /// Destroys all resources created through the interop API and releases interop structures.
    /// </summary>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Destroy))]
    public static void Destroy()
    {
        if (bufferPools == null || bufferPools == null || simulations == null)
        {
            throw new InvalidOperationException("Interop structures are not initialized; cannot destroy anything.");
        }
        for (int i = 0; i < bufferPools.Capacity; ++i)
        {
            var pool = bufferPools[i];
            if (pool != null)
            {
                pool.Clear();
            }
        }
        bufferPools = null;
        //The only resources held by the simulations that need to be released were allocated from the buffer pools, which we just destroyed. Nothing left to do!
        simulations = null;

        for (int i = 0; i < threadDispatchers.Capacity; ++i)
        {
            var dispatcher = threadDispatchers[i];
            if (dispatcher != null)
            {
                dispatcher.Dispose();
            }
        }
        threadDispatchers = null;
    }



    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetSIMDWidth))]
    public static SIMDWidth GetSIMDWidth()
    {
        //This will have to change later, but it technically works now.
        return Vector<float>.Count switch { 16 => SIMDWidth.SIMD512, 8 => SIMDWidth.SIMD256, _ => SIMDWidth.SIMD128 };
    }

    /// <summary>
    /// Gets the number of threads exposed by the operating system on this platform. Cores with SMT can show as having multiple threads.
    /// </summary>
    /// <returns>Number of threads exposed by the operating system on this platform.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetPlatformThreadCount))]
    public static int GetPlatformThreadCount()
    {
        return Environment.ProcessorCount;
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
    [return: TypeName(BufferPoolName)]
    public static InstanceHandle CreateBufferPool(int minimumBlockAllocationSize = 131072, int expectedUsedSlotCountPerPool = 16)
    {
        return bufferPools.Add(new BufferPool(minimumBlockAllocationSize, expectedUsedSlotCountPerPool));
    }

    /// <summary>
    /// Releases all allocations held by the buffer pool. The buffer pool remains in a usable state.
    /// </summary>
    /// <param name="handle">Buffer pool to clear.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ClearBufferPool))]
    public static void ClearBufferPool([TypeName(BufferPoolName)] InstanceHandle handle)
    {
        bufferPools[handle].Clear();
    }

    /// <summary>
    /// Releases all allocations held by the buffer pool and releases the buffer pool reference. The handle is invalidated.
    /// </summary>
    /// <param name="handle">Buffer pool to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyBufferPool))]
    public static void DestroyBufferPool([TypeName(BufferPoolName)] InstanceHandle handle)
    {
        bufferPools[handle].Clear();
        bufferPools.Remove(handle);
    }

    /// <summary>
    /// Allocates a buffer from the buffer pool of the given size.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
    /// <param name="sizeInBytes">Size of the buffer to allocate in bytes.</param>
    /// <returns>Allocated buffer.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Allocate))]
    [return: TypeName("ByteBuffer")]
    public static Buffer<byte> Allocate([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, int sizeInBytes)
    {
        bufferPools[bufferPoolHandle].Take<byte>(sizeInBytes, out var buffer);
        return buffer;
    }

    /// <summary>
    /// Allocates a buffer from the buffer pool with at least the given size.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
    /// <param name="sizeInBytes">Size of the buffer to allocate in bytes.</param>
    /// <returns>Allocated buffer.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AllocateAtLeast))]
    [return: TypeName("ByteBuffer")]
    public static Buffer<byte> AllocateAtLeast([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, int sizeInBytes)
    {
        bufferPools[bufferPoolHandle].TakeAtLeast<byte>(sizeInBytes, out var buffer);
        return buffer;
    }

    /// <summary>
    /// Resizes a buffer from the buffer pool to the given size, reallocating if necessary.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
    /// <param name="buffer">Buffer to resize.</param>
    /// <param name="newSizeInBytes">Target size of the buffer to allocate in bytes.</param>
    /// <param name="copyCount">Number of bytes to copy from the old buffer into the new buffer.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Resize))]
    public static unsafe void Resize([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, [TypeName("ByteBuffer*")] Buffer<byte>* buffer, int newSizeInBytes, int copyCount)
    {
        bufferPools[bufferPoolHandle].Resize(ref *buffer, newSizeInBytes, copyCount);
    }

    /// <summary>
    /// Resizes a buffer from the buffer pool to at least the given size, reallocating if necessary.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
    /// <param name="buffer">Buffer to resize.</param>
    /// <param name="targetSizeInBytes">Target size of the buffer to allocate in bytes.</param>
    /// <param name="copyCount">Number of bytes to copy from the old buffer into the new buffer.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ResizeToAtLeast))]
    public static unsafe void ResizeToAtLeast([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, [TypeName("ByteBuffer*")] Buffer<byte>* buffer, int targetSizeInBytes, int copyCount)
    {
        bufferPools[bufferPoolHandle].ResizeToAtLeast(ref *buffer, targetSizeInBytes, copyCount);
    }

    /// <summary>
    /// Returns a buffer to the buffer pool.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to return the buffer to.</param>
    /// <param name="buffer">Buffer to return to the pool.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Deallocate))]
    public unsafe static void Deallocate([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, [TypeName("ByteBuffer*")] Buffer<byte>* buffer)
    {
        bufferPools[bufferPoolHandle].Return(ref *buffer);
    }

    /// <summary>
    /// Returns a buffer to the buffer pool by its id.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to return the buffer to.</param>
    /// <param name="bufferId">Id of the buffer to return to the pool.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DeallocateById))]
    public unsafe static void DeallocateById([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle, int bufferId)
    {
        bufferPools[bufferPoolHandle].ReturnUnsafely(bufferId);
    }


    /// <summary>
    /// Creates a new thread dispatcher.
    /// </summary>
    /// <param name="threadCount">Number of threads to use within the thread dispatcher.</param>
    /// <param name="threadPoolAllocationBlockSize">Minimum size in bytes of blocks allocated in per-thread buffer pools. Allocations requiring more space can result in larger block sizes, but no pools will allocate smaller blocks.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateThreadDispatcher))]
    [return: TypeName(ThreadDispatcherName)]
    public static InstanceHandle CreateThreadDispatcher(int threadCount, int threadPoolAllocationBlockSize = 16384)
    {
        return threadDispatchers.Add(new ThreadDispatcher(threadCount, threadPoolAllocationBlockSize));
    }

    /// <summary>
    /// Releases all resources held by a thread dispatcher and invalidates its handle.
    /// </summary>
    /// <param name="handle">Thread dispatcher to destroy.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroyThreadDispatcher))]
    public static void DestroyThreadDispatcher([TypeName(ThreadDispatcherName)] InstanceHandle handle)
    {
        threadDispatchers[handle].Dispose();
        threadDispatchers.Remove(handle);
    }

    /// <summary>
    /// Releases all resources held by a thread dispatcher and invalidates its handle.
    /// </summary>
    /// <param name="handle">Thread dispatcher to check the thread count of.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetThreadCount))]
    public static int GetThreadCount([TypeName(ThreadDispatcherName)] InstanceHandle handle)
    {
        return threadDispatchers[handle].ThreadCount;
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
    /// <param name="narrowPhaseCallbacks">Narrow phase callbacks to be invoked by the simulation.</param>
    /// <param name="poseIntegratorCallbacks">Pose integration state and callbacks to be invoked by the simulation.</param>
    /// <param name="solveDescriptionInterop">Defines velocity iteration count and substep counts for the simulation's solver.</param>
    /// <param name="initialAllocationSizes">Initial capacities to allocate within the simulation.</param>
    /// <returns></returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(CreateSimulation))]
    [return: TypeName(SimulationName)]
    public unsafe static InstanceHandle CreateSimulation(
        [TypeName(BufferPoolName)] InstanceHandle bufferPool,
        [TypeName("NarrowPhaseCallbacks")] NarrowPhaseCallbacksInterop narrowPhaseCallbacks,
        [TypeName("PoseIntegratorCallbacks")] PoseIntegratorCallbacksInterop poseIntegratorCallbacks,
        [TypeName("SolveDescription")] SolveDescriptionInterop solveDescriptionInterop, SimulationAllocationSizes initialAllocationSizes)
    {
        var solveDescription = new SolveDescription
        {
            VelocityIterationCount = solveDescriptionInterop.VelocityIterationCount,
            SubstepCount = solveDescriptionInterop.SubstepCount,
            FallbackBatchThreshold = solveDescriptionInterop.FallbackBatchThreshold,
            VelocityIterationScheduler = solveDescriptionInterop.VelocityIterationScheduler != null ? Marshal.GetDelegateForFunctionPointer<SubstepVelocityIterationScheduler>((IntPtr)solveDescriptionInterop.VelocityIterationScheduler) : null
        };
        var narrowPhaseCallbacksImpl = new NarrowPhaseCallbacks
        {
            InitializeFunction = narrowPhaseCallbacks.InitializeFunction,
            DisposeFunction = narrowPhaseCallbacks.DisposeFunction,
            AllowContactGenerationFunction = narrowPhaseCallbacks.AllowContactGenerationFunction,
            AllowContactGenerationBetweenChildrenFunction = narrowPhaseCallbacks.AllowContactGenerationBetweenChildrenFunction,
            ConfigureConvexContactManifoldFunction = narrowPhaseCallbacks.ConfigureConvexContactManifoldFunction,
            ConfigureNonconvexContactManifoldFunction = narrowPhaseCallbacks.ConfigureNonconvexContactManifoldFunction,
            ConfigureChildContactManifoldFunction = narrowPhaseCallbacks.ConfigureChildContactManifoldFunction
        };
        return CreateSimulation(bufferPools[bufferPool], narrowPhaseCallbacksImpl, poseIntegratorCallbacks, solveDescription, initialAllocationSizes);
    }

    /// <summary>
    /// Destroys a simulation and invalidates its handle.
    /// </summary>
    /// <param name="handle">Simulation to destroy.</param>

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(DestroySimulation))]
    public static unsafe void DestroySimulation([TypeName(SimulationName)] InstanceHandle handle)
    {
        simulations[handle].Dispose();
        simulations.Remove(handle);
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddBody))]
    public unsafe static BodyHandle AddBody([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyDescription bodyDescription)
    {
        return simulations[simulationHandle].Bodies.Add(bodyDescription);
    }
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(RemoveBody))]
    public unsafe static void RemoveBody([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle)
    {
        simulations[simulationHandle].Bodies.Remove(bodyHandle);
    }

    /// <summary>
    /// Gets a pointer to the dynamic state associated with a body. Includes pose, velocity, and inertia.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a body's state from.</param>
    /// <param name="bodyHandle">Body handle to pull data about.</param>
    /// <returns>Pointer to the body's dynamic state.</returns>
    /// <remarks>This is a direct pointer. The memory location associated with a body can move other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodyDynamics))]
    public unsafe static BodyDynamics* GetBodyDynamics([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle)
    {
        return (BodyDynamics*)Unsafe.AsPointer(ref simulations[simulationHandle].Bodies[bodyHandle].Dynamics);
    }

    /// <summary>
    /// Gets a pointer to the collidable associated with a body.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a body's state from.</param>
    /// <param name="bodyHandle">Body handle to pull data about.</param>
    /// <returns>Pointer to the body's collidable.</returns>
    /// <remarks>This is a direct pointer. The memory location associated with a body can move if other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodyCollidable))]
    public unsafe static Collidable* GetBodyCollidable([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle)
    {
        return (Collidable*)Unsafe.AsPointer(ref simulations[simulationHandle].Bodies[bodyHandle].Collidable);
    }

    /// <summary>
    /// Gets a pointer to the activity state associated with a body.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a body's state from.</param>
    /// <param name="bodyHandle">Body handle to pull data about.</param>
    /// <returns>Pointer to the body's activity state.</returns>
    /// <remarks>This is a direct pointer. The memory location associated with a body can move if other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodyActivity))]
    public unsafe static BodyActivity* GetBodyActivity([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle)
    {
        return (BodyActivity*)Unsafe.AsPointer(ref simulations[simulationHandle].Bodies[bodyHandle].Activity);
    }

    /// <summary>
    /// Gets a pointer to the list of constraints associated with a body.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a body's state from.</param>
    /// <param name="bodyHandle">Body handle to pull data about.</param>
    /// <returns>Pointer to the body's constraint list.</returns>
    /// <remarks>This is a direct pointer. The memory location associated with a body can move if other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodyConstraints))]
    [return: TypeName("QuickList<BodyConstraintReference>*")]
    public unsafe static QuickList<ConstraintReference>* GetBodyConstraints([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle)
    {
        return (QuickList<ConstraintReference>*)Unsafe.AsPointer(ref simulations[simulationHandle].Bodies[bodyHandle].Constraints);
    }

    /// <summary>
    /// Gets a description of a body.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a body's state from.</param>
    /// <param name="bodyHandle">Body handle to pull data about.</param>
    /// <returns>Description of a body.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodyDescription))]
    public unsafe static BodyDescription GetBodyDescription([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle)
    {
        return simulations[simulationHandle].Bodies.GetDescription(bodyHandle);
    }

    /// <summary>
    /// Applies a description to a body.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a body's state from.</param>
    /// <param name="bodyHandle">Body handle to pull data about.</param>
    /// <param name="description">Description to apply to the body.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ApplyBodyDescription))]
    public unsafe static void ApplyBodyDescription([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle, BodyDescription description)
    {
        simulations[simulationHandle].Bodies.ApplyDescription(bodyHandle, description);
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(AddStatic))]
    public unsafe static StaticHandle AddStatic([TypeName(SimulationName)] InstanceHandle simulationHandle, StaticDescription staticDescription)
    {
        return simulations[simulationHandle].Statics.Add(staticDescription);
    }
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(RemoveStatic))]
    public unsafe static void RemoveStatic([TypeName(SimulationName)] InstanceHandle simulationHandle, StaticHandle staticHandle)
    {
        simulations[simulationHandle].Statics.Remove(staticHandle);
    }
    /// <summary>
    /// Gets a pointer to data associated with a static.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a static's state from.</param>
    /// <param name="staticHandle">Static handle to pull data about.</param>
    /// <returns>Pointer to the static's data.</returns>
    /// <remarks>This is a direct pointer. The memory location associated with a static can move if other statics are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetStatic))]
    public unsafe static Static* GetStatic([TypeName(SimulationName)] InstanceHandle simulationHandle, StaticHandle staticHandle)
    {
        return (Static*)Unsafe.AsPointer(ref simulations[simulationHandle].Statics.GetDirectReference(staticHandle));
    }

    /// <summary>
    /// Gets a static's description.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a static's state from.</param>
    /// <param name="staticHandle">Static handle to pull data about.</param>
    /// <returns>Description of the static..</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetStaticDescription))]
    public unsafe static StaticDescription GetStaticDescription([TypeName(SimulationName)] InstanceHandle simulationHandle, StaticHandle staticHandle)
    {
        return simulations[simulationHandle].Statics.GetDescription(staticHandle);
    }

    /// <summary>
    /// Applies a description to a static.
    /// </summary>
    /// <param name="simulationHandle">Simulation to pull a static's state from.</param>
    /// <param name="staticHandle">Static handle to pull data about.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(ApplyStaticDescription))]
    public unsafe static void ApplyStaticDescription([TypeName(SimulationName)] InstanceHandle simulationHandle, StaticHandle staticHandle, StaticDescription description)
    {
        simulations[simulationHandle].Statics.ApplyDescription(staticHandle, description);
    }

    /// <summary>
    /// Steps the simulation forward a single time.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to step.</param>
    /// <param name="dt">Duration of the timestep.</param>
    /// <param name="threadDispatcherHandle">Handle of the thread dispatcher to use, if any. Can be a null reference.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(Timestep))]
    public unsafe static void Timestep([TypeName(SimulationName)] InstanceHandle simulationHandle, float dt, [TypeName(ThreadDispatcherName)] InstanceHandle threadDispatcherHandle = new())
    {
        var threadDispatcher = threadDispatcherHandle.Null ? null : threadDispatchers[threadDispatcherHandle];
        simulations[simulationHandle].Timestep(dt, threadDispatcher);
    }

    /// <summary>
    /// Grabs a collidable's bounding boxes in the broad phase.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
    /// <param name="bodyHandle">Body to pull bounding box data about.</param>
    /// <param name="min">Minimum bounds of the collidable's bounding box.</param>
    /// <param name="max">Maximum bounds of the collidable's bounding box.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodyBoundingBoxInBroadPhase))]
    public unsafe static void GetBodyBoundingBoxInBroadPhase([TypeName(SimulationName)] InstanceHandle simulationHandle, BodyHandle bodyHandle, Vector3* min, Vector3* max)
    {
        simulations[simulationHandle].Bodies[bodyHandle].GetBoundsReferencesFromBroadPhase(out var minPointer, out var maxPointer);
        *min = *minPointer;
        *max = *maxPointer;
    }

    /// <summary>
    /// Grabs a collidable's bounding boxes in the broad phase.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
    /// <param name="staticHandle">Static to pull bounding box data about.</param>
    /// <param name="min">Minimum bounds of the collidable's bounding box.</param>
    /// <param name="max">Maximum bounds of the collidable's bounding box.</param>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetStaticBoundingBoxInBroadPhase))]
    public unsafe static void GetStaticBoundingBoxInBroadPhase([TypeName(SimulationName)] InstanceHandle simulationHandle, StaticHandle staticHandle, Vector3* min, Vector3* max)
    {
        simulations[simulationHandle].Statics[staticHandle].GetBoundsReferencesFromBroadPhase(out var minPointer, out var maxPointer);
        *min = *minPointer;
        *max = *maxPointer;
    }

    /// <summary>
    /// Gets the mapping from body handles to the body's location in storage.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
    /// <param name="bodyHandleToIndexMapping">Mapping from a body handle to the body's memory location.</param>
    /// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodyHandleToLocationMapping))]
    public unsafe static void GetBodyHandleToLocationMapping([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName("Buffer<BodyMemoryLocation>*")] Buffer<BodyMemoryLocation>* bodyHandleToIndexMapping)
    {
        *bodyHandleToIndexMapping = simulations[simulationHandle].Bodies.HandleToLocation;
    }

    /// <summary>
    /// Gets the body sets for a simulation. Slot 0 is the active set. Subsequent sets are sleeping. Not every slot beyond slot 0 is filled.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
    /// <param name="bodySets">Mapping from a body handle to the body's memory location.</param>
    /// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetBodySets))]
    public unsafe static void GetBodySets([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName("Buffer<BodySet>*")] Buffer<BodySet>* bodySets)
    {
        *bodySets = simulations[simulationHandle].Bodies.Sets;
    }

    /// <summary>
    /// Gets the mapping from body handles to the body's location in storage.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
    /// <param name="staticHandleToIndexMapping">Mapping from a static handle to the static's memory location.</param>
    /// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetStaticHandleToLocationMapping))]
    public unsafe static void GetStaticHandleToLocationMapping([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName("Buffer<int32_t>*")] Buffer<int>* staticHandleToIndexMapping)
    {
        *staticHandleToIndexMapping = simulations[simulationHandle].Statics.HandleToIndex;
    }

    /// <summary>
    /// Gets the statics set for a simulation.
    /// </summary>
    /// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
    /// <param name="statics">The set of all statics within a simulation.</param>
    /// <param name="count">Number of statics in the simulation.</param>
    /// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it. The count is a snapshot.</remarks>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetStatics))]
    public unsafe static void GetStatics([TypeName(SimulationName)] InstanceHandle simulationHandle, [TypeName("Buffer<Static>*")] Buffer<Static>* statics, [TypeName("int32_t*")] int* count)
    {
        *statics = simulations[simulationHandle].Statics.StaticsBuffer;
        *count = simulations[simulationHandle].Statics.Count;
    }

    /// <summary>
    /// Computes the total number of bytes allocated from native memory in this buffer pool.
    /// Includes allocated memory regardless of whether it currently has outstanding references.
    /// </summary>
    /// <param name="bufferPoolHandle">Buffer pool to check the allocation size of.</param>
    /// <returns>Total number of bytes allocated from native memory in this buffer pool.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetAllocatedMemorySizeInPool))]
    public unsafe static ulong GetAllocatedMemorySizeInPool([TypeName(BufferPoolName)] InstanceHandle bufferPoolHandle)
    {
        return bufferPools[bufferPoolHandle].GetTotalAllocatedByteCount();
    }

    /// <summary>
    /// Computes the total number of bytes allocated from native memory in a dispatcher's per-thread pools.
    /// Includes allocated memory regardless of whether it currently has outstanding references.
    /// </summary>
    /// <param name="threadDispatcherHandle">Thread dispatcher to check allocations for.</param>
    /// <returns>Total number of bytes allocated from native memory in this thread dispatcher's per-thread pool.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetAllocatedMemorySizeInThreadDispatcher))]
    public unsafe static ulong GetAllocatedMemorySizeInThreadDispatcher([TypeName(ThreadDispatcherName)] InstanceHandle threadDispatcherHandle)
    {
        ulong sum = 0;
        var dispatcher = threadDispatchers[threadDispatcherHandle];
        for (int i = 0; i < dispatcher.ThreadCount; ++i)
        {
            sum += dispatcher.WorkerPools[i].GetTotalAllocatedByteCount();
        }
        return sum;
    }

    /// <summary>
    /// Estimates the number of bytes managed by the garbage collector.
    /// </summary>
    /// <returns>Estimated number of bytes allocated from managed memory.</returns>
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) }, EntryPoint = FunctionNamePrefix + nameof(GetGCAllocatedMemorySize))]
    public unsafe static ulong GetGCAllocatedMemorySize()
    {
        return (ulong)GC.GetTotalMemory(false);
    }




}
