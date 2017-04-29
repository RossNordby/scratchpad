using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{
    public struct ManifoldContactData
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector<float> PenetrationDepth;
    }
    public struct ContactManifold4PrestepData
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide OffsetA2;
        public Vector3Wide OffsetA3;
        public Vector3Wide OffsetB0;
        public Vector3Wide OffsetB1;
        public Vector3Wide OffsetB2;
        public Vector3Wide OffsetB3;
        public Vector<float> FrictionCoefficient;        
        //In a convex manifold, all contacts share the same normal and tangents.
        public QuaternionWide SurfaceBasis;
        //All contacts also share the spring settings.
        public SpringSettings SpringSettings;
        public Vector<float> PenetrationDepth0;
        public Vector<float> PenetrationDepth1;
        public Vector<float> PenetrationDepth2;
        public Vector<float> PenetrationDepth3;
    }

    public struct ContactManifold4AccumulatedImpulses
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration0;
        public Vector<float> Penetration1;
        public Vector<float> Penetration2;
        public Vector<float> Penetration3;
        public Vector<float> Twist;
    }
    //The key observation here is that we have 7DOFs worth of constraints that all share the exact same bodies.
    //Despite the potential premultiplication optimizations, we focus on a few big wins:
    //1) Sharing the inverse mass for the impulse->velocity projection across all constraints.
    //2) Sharing the normal as much as possible.
    //3) Resorting to iteration-side redundant calculation if it reduces memory bandwidth.
    //This is expected to slow down the single threaded performance when running on a 128 bit SIMD machine.
    //However, when using multiple threads, memory bandwidth very rapidly becomes a concern.
    //In fact, a hypothetical CLR and machine that supported AVX512 would hit memory bandwidth limits on the older implementation that used 2032 bytes per bundle for projection data...
    //on a single thread.

    public struct ContactManifold4Projection
    {
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
        public Vector<float> PremultipliedFrictionCoefficient;
        public QuaternionWide SurfaceBasis;
        public TangentFrictionProjection Tangent;
        public ContactPenetrationLimit4Projection Penetration;
        //Lever arms aren't included in the twist projection because the number of arms required varies independently of the twist projection itself.
        public Vector<float> LeverArm0;
        public Vector<float> LeverArm1;
        public Vector<float> LeverArm2;
        public Vector<float> LeverArm3;
        public TwistFrictionProjection Twist;
    }

    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact convex manifold constraints.
    /// </summary>
    public class ContactManifold4TypeBatch : TwoBodyTypeBatch<ContactManifold4PrestepData, ContactManifold4Projection, ContactManifold4AccumulatedImpulses>
    {
        public override void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref PrestepData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var projectionBase = ref Projection[0];

            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                //Some speculative compression options not (yet) pursued:
                //1) Store the surface basis in a compressed fashion. It could be stored within 32 bits by using standard compression schemes, but we lack the necessary
                //instructions to properly SIMDify the decode operation (e.g. shift). Even with the potential savings of 3 floats (relative to uncompressed storage), it would be questionable.
                //We could drop one of the four components of the quaternion and reconstruct it relatively easily- that would just require that the encoder ensures the W component is positive.
                //It would require a square root, but it might still be a net win. On an IVB, sqrt has a 7 cycle throughput. 4 bytes saved * 4 lanes = 16 bytes, which takes 
                //about 16 / 5.5GBps = 2.9ns, where 5.5 is roughly the per-core bandwidth on a 3770K. 7 cycles is only 2ns at 3.5ghz. 
                //There are a couple of other instructions necessary to decode, but sqrt is by far the heaviest; it's likely a net win.
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);

                GatherScatter.GatherInertia(bodyInertias, ref bodyReferences, out projection.InertiaA, out projection.InertiaB);
                //Be careful about the execution order here. It should be aligned with the prestep data layout to ensure prefetching works well.
                //(Note that the Vector3Wide operations cause some degree of non-sequential access. We COULD fix that, but it would get gross; we'd have to store OffsetAX0, OffsetAX1...
                //Probably another 1-3% prestep performance on the table there. Blegh.)
                Vector3Wide.Add(ref prestep.OffsetA0, ref prestep.OffsetA1, out var a01);
                Vector3Wide.Add(ref prestep.OffsetA2, ref prestep.OffsetA3, out var a23);
                Vector3Wide.Add(ref a01, ref a23, out var offsetToManifoldCenterA);
                var scale = new Vector<float>(0.25f);
                Vector3Wide.Scale(ref offsetToManifoldCenterA, ref scale, out offsetToManifoldCenterA);
                Vector3Wide.Add(ref prestep.OffsetB0, ref prestep.OffsetB1, out var b01);
                Vector3Wide.Add(ref prestep.OffsetB2, ref prestep.OffsetB3, out var b23);
                Vector3Wide.Add(ref b01, ref b23, out var offsetToManifoldCenterB);
                Vector3Wide.Scale(ref offsetToManifoldCenterB, ref scale, out offsetToManifoldCenterB);
                projection.PremultipliedFrictionCoefficient = scale * prestep.FrictionCoefficient;
                projection.SurfaceBasis = prestep.SurfaceBasis;
                Matrix3x3Wide.CreateFromQuaternion(ref prestep.SurfaceBasis, out var surfaceBasis);
                TangentFriction.Prestep(ref surfaceBasis.X, ref surfaceBasis.Z, ref offsetToManifoldCenterA, ref offsetToManifoldCenterB, ref projection.InertiaA, ref projection.InertiaB, out projection.Tangent);
                ContactPenetrationLimit4.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref surfaceBasis.Y, ref prestep, dt, inverseDt, out projection.Penetration);
                //Just assume the lever arms for B are the same. It's a good guess. (The only reason we computed the offset B is because we didn't want to go into world space.)
                Vector3Wide.Distance(ref prestep.OffsetA0, ref offsetToManifoldCenterA, out projection.LeverArm0);
                Vector3Wide.Distance(ref prestep.OffsetA1, ref offsetToManifoldCenterA, out projection.LeverArm1);
                Vector3Wide.Distance(ref prestep.OffsetA2, ref offsetToManifoldCenterA, out projection.LeverArm2);
                Vector3Wide.Distance(ref prestep.OffsetA3, ref offsetToManifoldCenterA, out projection.LeverArm3);
                TwistFriction.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref surfaceBasis.Y, out projection.Twist);
            }
        }
        public override void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            ref var projectionBase = ref Projection[0];
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                GatherScatter.GatherVelocities(bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
                Matrix3x3Wide.CreateFromQuaternion(ref projection.SurfaceBasis, out var surfaceBasis);
                TangentFriction.WarmStart(ref surfaceBasis.X, ref surfaceBasis.Z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                ContactPenetrationLimit4.WarmStart(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB,
                    ref surfaceBasis.Y,
                    ref accumulatedImpulses.Penetration0,
                    ref accumulatedImpulses.Penetration1,
                    ref accumulatedImpulses.Penetration2,
                    ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);
                TwistFriction.WarmStart(ref surfaceBasis.Y, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
            }
        }
        public override void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int exclusiveEndBundle)
        {
            ref var projectionBase = ref Projection[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);

                GatherScatter.GatherVelocities(bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);

                Matrix3x3Wide.CreateFromQuaternion(ref projection.SurfaceBasis, out var surfaceBasis);
                var maximumTangentImpulse = projection.PremultipliedFrictionCoefficient *
                    (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2 + accumulatedImpulses.Penetration3);
                TangentFriction.Solve(ref surfaceBasis.X, ref surfaceBasis.Z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                //Note that we solve the penetration constraints after the friction constraints. 
                //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
                //It's a pretty minor effect either way.
                ContactPenetrationLimit4.Solve(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB, ref surfaceBasis.Y,
                    ref accumulatedImpulses.Penetration0,
                    ref accumulatedImpulses.Penetration1,
                    ref accumulatedImpulses.Penetration2,
                    ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);

                var maximumTwistImpulse = projection.PremultipliedFrictionCoefficient * (
                    accumulatedImpulses.Penetration0 * projection.LeverArm0 +
                    accumulatedImpulses.Penetration1 * projection.LeverArm1 +
                    accumulatedImpulses.Penetration2 * projection.LeverArm2 +
                    accumulatedImpulses.Penetration3 * projection.LeverArm3);
                TwistFriction.Solve(ref surfaceBasis.Y, ref projection.InertiaA, ref projection.InertiaB, ref projection.Twist, ref maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
            }
        }

    }
}
