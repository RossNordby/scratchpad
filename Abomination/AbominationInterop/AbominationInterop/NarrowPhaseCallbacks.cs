using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace AbominationInterop;

[StructLayout(LayoutKind.Sequential)]
public unsafe struct NarrowPhaseCallbacksInterop
{
    public delegate*<InstanceHandle, void> InitializeFunction;
    public delegate*<InstanceHandle, void> DisposeFunction;
    public delegate*<InstanceHandle, int, CollidableReference, CollidableReference, float*, byte> AllowContactGenerationFunction;
    public delegate*<InstanceHandle, int, CollidablePair, int, int, byte> AllowContactGenerationBetweenChildrenFunction;
    public delegate*<InstanceHandle, int, CollidablePair, ConvexContactManifold*, PairMaterialProperties*, byte> ConfigureConvexContactManifoldFunction;
    public delegate*<InstanceHandle, int, CollidablePair, NonconvexContactManifold*, PairMaterialProperties*, byte> ConfigureNonconvexContactManifoldFunction;
    public delegate*<InstanceHandle, int, CollidablePair, int, int, ConvexContactManifold*, byte> ConfigureChildContactManifoldFunction;
}

public unsafe struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    public delegate*<InstanceHandle, void> InitializeFunction;
    public delegate*<InstanceHandle, void> DisposeFunction;
    public delegate*<InstanceHandle, int, CollidableReference, CollidableReference, float*, byte> AllowContactGenerationFunction;
    public delegate*<InstanceHandle, int, CollidablePair, int, int, byte> AllowContactGenerationBetweenChildrenFunction;
    public delegate*<InstanceHandle, int, CollidablePair, ConvexContactManifold*, PairMaterialProperties*, byte> ConfigureConvexContactManifoldFunction;
    public delegate*<InstanceHandle, int, CollidablePair, NonconvexContactManifold*, PairMaterialProperties*, byte> ConfigureNonconvexContactManifoldFunction;
    public delegate*<InstanceHandle, int, CollidablePair, int, int, ConvexContactManifold*, byte> ConfigureChildContactManifoldFunction;

    public InstanceHandle Simulation;

    public void Initialize(Simulation simulation)
    {
        //Note that we don't use this for the interop callbacks; we don't have a simulation handle yet, so the interop can't yet refer to the simulation!
    }
    public void Initialize(InstanceHandle simulation)
    {
        Simulation = simulation;
        if (InitializeFunction != null)
            InitializeFunction(simulation);
    }
    public void Dispose()
    {
        if (DisposeFunction != null)
            DisposeFunction(Simulation);
    }

    //Note that a number of these convert refs into pointers. These are safe; all such references originate on the stack or pinned memory.
    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {
        return AllowContactGenerationFunction(Simulation, workerIndex, a, b, (float*)Unsafe.AsPointer(ref speculativeMargin)) != 0;
    }

    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
    {
        return AllowContactGenerationBetweenChildrenFunction(Simulation, workerIndex, pair, childIndexA, childIndexB) != 0;
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
            return ConfigureConvexContactManifoldFunction(Simulation, workerIndex, pair, (ConvexContactManifold*)Unsafe.AsPointer(ref manifold), pairMaterialPointer) != 0;
        }
        else
        {
            return ConfigureNonconvexContactManifoldFunction(Simulation, workerIndex, pair, (NonconvexContactManifold*)Unsafe.AsPointer(ref manifold), pairMaterialPointer) != 0;
        }
    }

    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
    {
        return ConfigureChildContactManifoldFunction(Simulation, workerIndex, pair, childIndexA, childIndexB, (ConvexContactManifold*)Unsafe.AsPointer(ref manifold)) != 0;
    }
}
