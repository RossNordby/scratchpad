using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{

    //The key observation here is that we have 7DOFs worth of constraints that all share the exact same bodies.
    //Despite the potential premultiplication optimizations, we focus on a few big wins:
    //1) Sharing the inverse mass for the impulse->velocity projection across all constraints.
    //2) Sharing the normal as much as possible.
    //3) Resorting to iteration-side redundant calculation if it reduces memory bandwidth.
    //This is expected to slow down the single threaded performance when running on a 128 bit SIMD machine.
    //However, when using multiple threads, memory bandwidth very rapidly becomes a concern.
    //In fact, a hypothetical CLR and machine that supported AVX512 would hit memory bandwidth limits on the older implementation that used 2032 bytes per bundle for projection data...
    //on a single thread.

    public struct ContactManifold4ProjectionCompressed
    {
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
        public Vector<float> PremultipliedFrictionCoefficient;
        public Vector3Wide Normal;
        public TangentFrictionProjectionCompressed Tangent;
        public ContactPenetrationLimit4ProjectionCompressed Penetration;
        //Lever arms aren't included in the twist projection because the number of arms required varies independently of the twist projection itself.
        public Vector<float> LeverArm0;
        public Vector<float> LeverArm1;
        public Vector<float> LeverArm2;
        public Vector<float> LeverArm3;
        public TwistFrictionProjectionCompressed Twist;
    }

    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact convex manifold constraints.
    /// </summary>
    public class ContactManifold4TypeBatch : TwoBodyTypeBatch<ContactManifold4PrestepData, ContactManifold4ProjectionCompressed, ContactManifold4AccumulatedImpulses>
    {
        public override void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref PrestepData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var projectionBase = ref Projection[0];

            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                //Some speculative compression options not (yet) pursued:
                //1) Store the normal+basis compressed as a 32 bit quaternion. (We don't have sufficient SIMD bitfiddling instructions available to make this reasonable.)
                //Compared to storing two full precision axes, it would save 5 floats per constraint. A single full precision quaternion is more doable and saves 2 floats per constraint.
                //We could drop one float and reconstruct with a normalize.
                //Quaternion->basis isn't supercheap (~24 ops), but saving 2 floats per constraint would reduce bundle size by 64 bytes on 256 bit SIMD.
                //When bandwidth constrained, 24 high throughput ops (add/mul) with good ILP could easily complete faster than loading 64 bytes. At a 6700K's peak bandwidth,
                //loading 64 bytes would take 8 cycles. In 8 cycles it could do 16 MULSS. Even if it doesn't turn out to be a great thing for a single thread, consider
                //multiple threads- where even 128 bit wide SIMD can easily hit bandwidth bottlenecks. And now think about the AVX512-including future.
                //(Note that the 2-axis requires reconstructing a tangent, which costs 6 mul and 3 sub anyway. If you loaded a full 3 axis basis, the difference is
                //5 floats- 160 bytes for 256 bit SIMD, which is enough time for 40 MULSS.)
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);

                GatherScatter.GatherInertia(bodyInertias, ref bodyReferences, out projection.InertiaA, out projection.InertiaB);
                projection.Normal = prestep.Normal;
                ContactPenetrationLimit4Compressed.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref prestep, dt, inverseDt, out projection.Penetration);
                TwistFrictionCompressed.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref prestep.Normal, out projection.Twist);
                Vector3Wide.Add(ref prestep.Contact0.OffsetA, ref prestep.Contact1.OffsetA, out var a01);
                Vector3Wide.Add(ref prestep.Contact2.OffsetA, ref prestep.Contact3.OffsetA, out var a23);
                Vector3Wide.Add(ref prestep.Contact0.OffsetB, ref prestep.Contact1.OffsetB, out var b01);
                Vector3Wide.Add(ref prestep.Contact2.OffsetB, ref prestep.Contact3.OffsetB, out var b23);
                Vector3Wide.Add(ref a01, ref a23, out var offsetToManifoldCenterA);
                Vector3Wide.Add(ref b01, ref b23, out var offsetToManifoldCenterB);
                var scale = new Vector<float>(0.25f);
                Vector3Wide.Scale(ref offsetToManifoldCenterA, ref scale, out offsetToManifoldCenterA);
                Vector3Wide.Scale(ref offsetToManifoldCenterB, ref scale, out offsetToManifoldCenterB);
                //Just assume the lever arms for B are the same. It's a good guess. (The only reason we computed the offset B is because we didn't want to go into world space.)
                Vector3Wide.Distance(ref prestep.Contact0.OffsetA, ref offsetToManifoldCenterA, out projection.LeverArm0);
                Vector3Wide.Distance(ref prestep.Contact1.OffsetA, ref offsetToManifoldCenterA, out projection.LeverArm1);
                Vector3Wide.Distance(ref prestep.Contact2.OffsetA, ref offsetToManifoldCenterA, out projection.LeverArm2);
                Vector3Wide.Distance(ref prestep.Contact3.OffsetA, ref offsetToManifoldCenterA, out projection.LeverArm3);
                projection.PremultipliedFrictionCoefficient = scale * prestep.FrictionCoefficient;
                TangentFrictionCompressed.Prestep(ref prestep.Normal, ref prestep.TangentX, ref offsetToManifoldCenterA, ref offsetToManifoldCenterB, ref projection.InertiaA, ref projection.InertiaB, out projection.Tangent);
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
                TangentFrictionCompressed.WarmStart(ref projection.Normal, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                ContactPenetrationLimit4Compressed.WarmStart(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB,
                    ref projection.Normal,
                    ref accumulatedImpulses.Penetration0,
                    ref accumulatedImpulses.Penetration1,
                    ref accumulatedImpulses.Penetration2,
                    ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);
                TwistFrictionCompressed.WarmStart(ref projection.Normal, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
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

                var maximumTangentImpulse = projection.PremultipliedFrictionCoefficient *
                    (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2 + accumulatedImpulses.Penetration3);
                TangentFrictionCompressed.Solve(ref projection.Normal, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                //Note that we solve the penetration constraints after the friction constraints. 
                //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
                //It's a pretty minor effect either way.
                ContactPenetrationLimit4Compressed.Solve(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB, ref projection.Normal,
                    ref accumulatedImpulses.Penetration0,
                    ref accumulatedImpulses.Penetration1,
                    ref accumulatedImpulses.Penetration2,
                    ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);

                var maximumTwistImpulse = projection.PremultipliedFrictionCoefficient * (
                    accumulatedImpulses.Penetration0 * projection.LeverArm0 +
                    accumulatedImpulses.Penetration1 * projection.LeverArm1 +
                    accumulatedImpulses.Penetration2 * projection.LeverArm2 +
                    accumulatedImpulses.Penetration3 * projection.LeverArm3);
                TwistFrictionCompressed.Solve(ref projection.Normal, ref projection.InertiaA, ref projection.InertiaB, ref projection.Twist, ref maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
            }
        }

    }
}
