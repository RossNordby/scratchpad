using BepuPhysics;
using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace AbominationInterop;

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
