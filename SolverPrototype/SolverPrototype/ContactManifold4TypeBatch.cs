using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct ManifoldContactData
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector<float> PenetrationDepth;
    }
    public struct ContactManifold4PrestepData
    {
        //In a convex manifold, all contacts share the same normal.
        public Vector3Wide Normal;
        public ManifoldContactData Contact0;
        public ManifoldContactData Contact1;
        public ManifoldContactData Contact2;
        public ManifoldContactData Contact3;
        //All contacts also share the spring settings.
        public SpringSettings SpringSettings;
        public Vector3Wide TangentX;
        public Vector3Wide TangentY;
        public Vector<float> FrictionCoefficient;
    }
    public struct ContactManifold4Projection
    {
        public Vector<float> LeverArm0;
        public Vector<float> LeverArm1;
        public Vector<float> LeverArm2;
        public Vector<float> LeverArm3;
        public Vector<float> PremultipliedFrictionCoefficient;
        public TwistFrictionProjection Twist;
        public TangentFrictionProjection Tangent;
        public ContactPenetrationLimit4Projection Penetration;
    }
    public struct ContactManifold4Unprojection
    {
        public TwistFrictionUnprojection Twist;
        public TangentFrictionUnprojection Tangent;
        public ContactPenetrationLimit4Unprojection Penetration;
    }

    public struct ContactManifold4AccumulatedImpulses
    {
        public Vector2Wide Tangent;
        public Vector<float> Twist;
        public Vector<float> Penetration0;
        public Vector<float> Penetration1;
        public Vector<float> Penetration2;
        public Vector<float> Penetration3;
    }
    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact convex manifold constraints.
    /// </summary>
    public class ContactManifold4TypeBatch : TypeBatch<BodyReferences, ContactManifold4PrestepData, ContactManifold4Projection, ContactManifold4Unprojection, ContactManifold4AccumulatedImpulses>
    {

        public override void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt, int startBundle, int endBundle)
        {
            ref var prestepBase = ref PrestepData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var projectionBase = ref Projection[0];
            ref var unprojectionBase = ref Unprojection[0];
            for (int i = startBundle; i < endBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var unprojection = ref Unsafe.Add(ref unprojectionBase, i);

                GatherScatter.GatherInertia(bodyInertias, ref bodyReferences, out var inertiaA, out var inertiaB);
                ContactPenetrationLimit4.Prestep(ref inertiaA, ref inertiaB, ref prestep, dt, inverseDt, out projection.Penetration, out unprojection.Penetration);
                TwistFriction.Prestep(ref inertiaA, ref inertiaB, ref prestep.Normal, out projection.Twist, out unprojection.Twist);
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
                TangentFriction.ComputeJacobians(ref prestep.TangentX, ref prestep.TangentY, ref offsetToManifoldCenterA, ref offsetToManifoldCenterB, out var tangentJacobians);
                TangentFriction.Prestep(ref inertiaA, ref inertiaB, ref tangentJacobians, out projection.Tangent, out unprojection.Tangent);
            }
        }
        public override void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            ref var unprojectionBase = ref Unprojection[0];
            for (int i = startBundle; i < endBundle; ++i)
            {
                ref var unprojection = ref Unsafe.Add(ref unprojectionBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                GatherScatter.GatherVelocities(bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
                ContactPenetrationLimit4.WarmStart(ref unprojection.Penetration,
                    ref accumulatedImpulses.Penetration0,
                    ref accumulatedImpulses.Penetration1,
                    ref accumulatedImpulses.Penetration2,
                    ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);
                TwistFriction.WarmStart(ref unprojection.Twist, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
                TangentFriction.WarmStart(ref unprojection.Tangent, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
            }
        }
        public override void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            ref var projectionBase = ref Projection[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            for (int i = startBundle; i < endBundle; ++i)
            {
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                GatherScatter.GatherVelocities(bodyVelocities, ref BodyReferences[i], out var wsvA, out var wsvB);
                var maximumImpulse = projection.PremultipliedFrictionCoefficient *
                    (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2 + accumulatedImpulses.Penetration3);
                TwistFriction.Solve(ref projection.Twist, ref maximumImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
                TangentFriction.Solve(ref projection.Tangent, ref maximumImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                //Note that we solve the penetration constraints after the friction constraints. 
                //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
                //It's a pretty minor effect either way.
                ContactPenetrationLimit4.Solve(ref projection.Penetration,
                    ref accumulatedImpulses.Penetration0,
                    ref accumulatedImpulses.Penetration1,
                    ref accumulatedImpulses.Penetration2,
                    ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);

                GatherScatter.ScatterVelocities(bodyVelocities, ref BodyReferences[i], ref wsvA, ref wsvB);
            }
        }

    }
}
