using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct ContactManifold4PrestepData
    {
        public ContactData Contact0;
        public ContactData Contact1;
        public ContactData Contact2;
        public ContactData Contact3;
        public Vector3Wide TangentX;
        public Vector3Wide TangentY;
        public Vector<float> FrictionCoefficient;
    }
    public struct ContactManifold4IterationData
    {
        public Vector<float> LeverArm0;
        public Vector<float> LeverArm1;
        public Vector<float> LeverArm2;
        public Vector<float> LeverArm3;
        public Vector<float> PremultipliedFrictionCoefficient;
        public TwistFrictionIterationData Twist;
        public TangentFrictionIterationData Tangent;
        public IterationData2Body1DOF Penetration0;
        public IterationData2Body1DOF Penetration1;
        public IterationData2Body1DOF Penetration2;
        public IterationData2Body1DOF Penetration3;
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
    public class ContactManifold4TypeBatch : TypeBatch<BodyReferences, ContactManifold4PrestepData, ContactManifold4IterationData, ContactManifold4AccumulatedImpulses>
    {

        public override void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt, int startBundle, int endBundle)
        {
            ref var prestepBase = ref PrestepData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var iterationBase = ref IterationData[0];
            for (int i = startBundle; i < endBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var iteration = ref Unsafe.Add(ref iterationBase, i);
                ContactPenetrationLimit.ComputeJacobiansAndError(ref prestep.Contact0, out var jacobians0, out var error0);
                ContactPenetrationLimit.ComputeJacobiansAndError(ref prestep.Contact1, out var jacobians1, out var error1);
                ContactPenetrationLimit.ComputeJacobiansAndError(ref prestep.Contact2, out var jacobians2, out var error2);
                ContactPenetrationLimit.ComputeJacobiansAndError(ref prestep.Contact3, out var jacobians3, out var error3);
                GatherScatter.GatherInertia(bodyInertias, ref bodyReferences, out var inertiaA, out var inertiaB);
                Inequality2Body1DOF.Prestep(ref inertiaA, ref inertiaB, ref jacobians0, ref prestep.Contact0.SpringSettings, ref error0, dt, inverseDt, out iteration.Penetration0);
                Inequality2Body1DOF.Prestep(ref inertiaA, ref inertiaB, ref jacobians1, ref prestep.Contact1.SpringSettings, ref error1, dt, inverseDt, out iteration.Penetration1);
                Inequality2Body1DOF.Prestep(ref inertiaA, ref inertiaB, ref jacobians2, ref prestep.Contact2.SpringSettings, ref error2, dt, inverseDt, out iteration.Penetration2);
                Inequality2Body1DOF.Prestep(ref inertiaA, ref inertiaB, ref jacobians3, ref prestep.Contact3.SpringSettings, ref error3, dt, inverseDt, out iteration.Penetration3);
                TwistFriction.Prestep(ref inertiaA, ref inertiaB, ref prestep.Contact0.Normal, out iteration.Twist);
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
                Vector3Wide.Distance(ref prestep.Contact0.OffsetA, ref offsetToManifoldCenterA, out iteration.LeverArm0);
                Vector3Wide.Distance(ref prestep.Contact1.OffsetA, ref offsetToManifoldCenterA, out iteration.LeverArm1);
                Vector3Wide.Distance(ref prestep.Contact2.OffsetA, ref offsetToManifoldCenterA, out iteration.LeverArm2);
                Vector3Wide.Distance(ref prestep.Contact3.OffsetA, ref offsetToManifoldCenterA, out iteration.LeverArm3);
                iteration.PremultipliedFrictionCoefficient = scale * prestep.FrictionCoefficient;
                TangentFriction.ComputeJacobians(ref prestep.TangentX, ref prestep.TangentY, ref offsetToManifoldCenterA, ref offsetToManifoldCenterB, out var tangentJacobians);
                TangentFriction.Prestep(ref inertiaA, ref inertiaB, ref tangentJacobians, out iteration.Tangent);
            }
        }
        public override void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            ref var iterationBase = ref IterationData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            for (int i = startBundle; i < endBundle; ++i)
            {
                ref var iteration = ref Unsafe.Add(ref iterationBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                GatherScatter.GatherVelocities(bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
                Inequality2Body1DOF.WarmStart(ref iteration.Penetration0, ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
                Inequality2Body1DOF.WarmStart(ref iteration.Penetration1, ref accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
                Inequality2Body1DOF.WarmStart(ref iteration.Penetration2, ref accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
                Inequality2Body1DOF.WarmStart(ref iteration.Penetration3, ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);
                TwistFriction.WarmStart(ref iteration.Twist, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
                TangentFriction.WarmStart(ref iteration.Tangent, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
            }
        }
        public override void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            ref var iterationBase = ref IterationData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
            for (int i = startBundle; i < endBundle; ++i)
            {
                ref var iteration = ref Unsafe.Add(ref iterationBase, i);
                ref var bodyReferences = ref Unsafe.Add(ref bodyReferencesBase, i);
                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
                GatherScatter.GatherVelocities(bodyVelocities, ref BodyReferences[i], out var wsvA, out var wsvB);
                var maximumImpulse = iteration.PremultipliedFrictionCoefficient * 
                    (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2 + accumulatedImpulses.Penetration3);
                TwistFriction.Solve(ref iteration.Twist, ref maximumImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
                TangentFriction.Solve(ref iteration.Tangent, ref maximumImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
                //Note that we solve the penetration constraints after the friction constraints. 
                //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
                //It's a pretty minor effect either way.
                Inequality2Body1DOF.Solve(ref iteration.Penetration0, ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
                Inequality2Body1DOF.Solve(ref iteration.Penetration1, ref accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
                Inequality2Body1DOF.Solve(ref iteration.Penetration2, ref accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
                Inequality2Body1DOF.Solve(ref iteration.Penetration3, ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref BodyReferences[i], ref wsvA, ref wsvB);
            }
        }

    }
}
