using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype
{

    public struct ContactPenetrationLimit4Jacobians
    {
        public Vector3Wide AngularA0;
        public Vector3Wide AngularB0;
        public Vector3Wide AngularA1;
        public Vector3Wide AngularB1;
        public Vector3Wide AngularA2;
        public Vector3Wide AngularB2;
        public Vector3Wide AngularA3;
        public Vector3Wide AngularB3;
    }

    public struct ContactPenetrationLimitWSVToCSI
    {
        public Vector<float> EffectiveMass;
        //Note that these are just the raw jacobians, no precomputation with the JT*EffectiveMass.
        public Vector3Wide AngularA;
        public Vector3Wide AngularB;
    }
    public struct ContactPenetrationLimitCSIToWSV
    {
        //Note that these angular components include the inverse inertia transform: wsv * (JT * M^-1)
        //The linear components (i.e. the normal) do not, favoring lower memory bandwidth.
        public Vector3Wide AngularA;
        public Vector3Wide AngularB;
    }
    /// <summary>
    /// Four convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public struct ContactPenetrationLimit4IterationData
    {
        //Note that all linear components of WSVToCSI are simply the normal vector times the effective mass (negated for B).
        //Storing only the linear jacobian (normal) and then multiplying by the effective mass (a scalar per contact) saves a bunch of stores and loads.
        public Vector3Wide Normal;
        public ContactPenetrationLimitWSVToCSI WSVToCSI0;
        public ContactPenetrationLimitWSVToCSI WSVToCSI1;
        public ContactPenetrationLimitWSVToCSI WSVToCSI2;
        public ContactPenetrationLimitWSVToCSI WSVToCSI3;

        public Vector<float> BiasImpulse0;
        public Vector<float> BiasImpulse1;
        public Vector<float> BiasImpulse2;
        public Vector<float> BiasImpulse3;
        //Recall that the softness impulse scale is independent of effective mass due to cancellation.
        public Vector<float> SoftnessImpulseScale;

        //Nothing above is accessed by the warm start. Should really try splitting it.


        //Again, note that the linear jacobian is shared (the contact manifold normal).
        //The M^-1 term for both bodies in reintroduced, but for linear terms it's only 2 scalars per lane, compared to 21 scalars for the redundant jacobians.

        public Vector<float> InverseMassA;
        public Vector<float> InverseMassB;
        public ContactPenetrationLimitCSIToWSV CSIToWSV0;
        public ContactPenetrationLimitCSIToWSV CSIToWSV1;
        public ContactPenetrationLimitCSIToWSV CSIToWSV2;
        public ContactPenetrationLimitCSIToWSV CSIToWSV3;

        //The first half would introduce 4 multiplies, and save 17 loads.
        //The second half would introduce 8 multiplies, and save 19 loads.
        //My guess is that it's worthwhile. Really need to collect some data.

    }


    public static class ContactPenetrationLimit4
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobiansAndError(
            ref Vector3Wide normal,
            ref ManifoldContactData contact0,
            ref ManifoldContactData contact1,
            ref ManifoldContactData contact2,
            ref ManifoldContactData contact3,
            out ContactPenetrationLimit4Jacobians jacobians,
            out Vector<float> error0,
            out Vector<float> error1,
            out Vector<float> error2,
            out Vector<float> error3)
        {
            //The contact penetration constraint takes the form:
            //dot(positionA + offsetA, N) >= dot(positionB + offsetB, N)
            //Or:
            //dot(positionA + offsetA, N) - dot(positionB + offsetB, N) >= 0
            //dot(positionA + offsetA - positionB - offsetB, N) >= 0
            //where positionA and positionB are the center of mass positions of the bodies offsetA and offsetB are world space offsets from the center of mass to the contact,
            //and N is a unit length vector calibrated to point from B to A. (The normal pointing direction is important; it changes the sign.)
            //In practice, we'll use the collision detection system's penetration depth instead of trying to recompute the error here.

            //So, treating the normal as constant, the velocity constraint is:
            //dot(d/dt(positionA + offsetA - positionB - offsetB), N) >= 0
            //dot(linearVelocityA + d/dt(offsetA) - linearVelocityB - d/dt(offsetB)), N) >= 0
            //The velocity of the offsets are defined by the angular velocity.
            //dot(linearVelocityA + angularVelocityA x offsetA - linearVelocityB - angularVelocityB x offsetB), N) >= 0
            //dot(linearVelocityA, N) + dot(angularVelocityA x offsetA, N) - dot(linearVelocityB, N) - dot(angularVelocityB x offsetB), N) >= 0
            //Use the properties of the scalar triple product:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) - dot(linearVelocityB, N) - dot(offsetB x N, angularVelocityB) >= 0
            //Bake in the negations:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(-offsetB x N, angularVelocityB) >= 0
            //A x B = -B x A:
            //dot(linearVelocityA, N) + dot(offsetA x N, angularVelocityA) + dot(linearVelocityB, -N) + dot(N x offsetB, angularVelocityB) >= 0
            //And there you go, the jacobians!
            //linearA: N
            //angularA: offsetA x N
            //linearB: -N
            //angularB: N x offsetB
            Vector3Wide.CrossWithoutOverlap(ref contact0.OffsetA, ref normal, out jacobians.AngularA0);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref contact0.OffsetB, out jacobians.AngularB0);
            Vector3Wide.CrossWithoutOverlap(ref contact1.OffsetA, ref normal, out jacobians.AngularA1);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref contact1.OffsetB, out jacobians.AngularB1);
            Vector3Wide.CrossWithoutOverlap(ref contact2.OffsetA, ref normal, out jacobians.AngularA2);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref contact2.OffsetB, out jacobians.AngularB2);
            Vector3Wide.CrossWithoutOverlap(ref contact3.OffsetA, ref normal, out jacobians.AngularA3);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref contact3.OffsetB, out jacobians.AngularB3);

            //Note that we leave the penetration depth as is, even when it's negative. Speculative contacts!
            error0 = contact0.PenetrationDepth;
            error1 = contact1.PenetrationDepth;
            error2 = contact2.PenetrationDepth;
            error3 = contact3.PenetrationDepth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal, ref ContactPenetrationLimit4Jacobians angularJacobians, ref SpringSettings springSettings,
            ref Vector<float> error0,
            ref Vector<float> error1,
            ref Vector<float> error2,
            ref Vector<float> error3, float dt, float inverseDt, out ContactPenetrationLimit4IterationData data)
        {
            //effective mass
            //Note that the linear components are all redundant due to the shared normal.
            //Also note that the J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since each constraint is 1DOF (scalar multiply commutativity).

            data.Normal = normal;
            Vector3Wide.Dot(ref normal, ref normal, out var normalLengthSquared);
            var linearA = normalLengthSquared * inertiaA.InverseMass;
            var linearB = normalLengthSquared * inertiaB.InverseMass;

            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularA0, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV0.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularB0, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV0.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV0.AngularA, ref angularJacobians.AngularA0, out var angularA0);
            Vector3Wide.Dot(ref data.CSIToWSV0.AngularB, ref angularJacobians.AngularB0, out var angularB0);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularA1, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV1.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularB1, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV1.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV1.AngularA, ref angularJacobians.AngularA1, out var angularA1);
            Vector3Wide.Dot(ref data.CSIToWSV1.AngularB, ref angularJacobians.AngularB1, out var angularB1);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularA2, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV2.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularB2, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV2.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV2.AngularA, ref angularJacobians.AngularA2, out var angularA2);
            Vector3Wide.Dot(ref data.CSIToWSV2.AngularB, ref angularJacobians.AngularB2, out var angularB2);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularA3, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV3.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularJacobians.AngularB3, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV3.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV3.AngularA, ref angularJacobians.AngularA3, out var angularA3);
            Vector3Wide.Dot(ref data.CSIToWSV3.AngularB, ref angularJacobians.AngularB3, out var angularB3);
            data.InverseMassA = inertiaA.InverseMass;
            data.InverseMassB = inertiaB.InverseMass;


            var linear = linearA + linearB;
            var effectiveMass0 = Vector<float>.One / (linear + angularA0 + angularB0);
            var effectiveMass1 = Vector<float>.One / (linear + angularA1 + angularB1);
            var effectiveMass2 = Vector<float>.One / (linear + angularA2 + angularB2);
            var effectiveMass3 = Vector<float>.One / (linear + angularA3 + angularB3);

            var frequencyDt = springSettings.NaturalFrequency * dt;
            var twiceDampingRatio = springSettings.DampingRatio * 2; //Could precompute.
            var extra = Vector<float>.One / (frequencyDt * (frequencyDt + twiceDampingRatio));
            var effectiveMassCFMScale = Vector<float>.One / (Vector<float>.One + extra);

            data.SoftnessImpulseScale = extra * effectiveMassCFMScale;

            var positionErrorToVelocity = springSettings.NaturalFrequency / (frequencyDt + twiceDampingRatio);
            
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            data.WSVToCSI0.EffectiveMass = effectiveMass0 * effectiveMassCFMScale;
            data.WSVToCSI0.AngularA = angularJacobians.AngularA0;
            data.WSVToCSI0.AngularB = angularJacobians.AngularB0;
            data.WSVToCSI1.EffectiveMass = effectiveMass1 * effectiveMassCFMScale;
            data.WSVToCSI1.AngularA = angularJacobians.AngularA1;
            data.WSVToCSI1.AngularB = angularJacobians.AngularB1;
            data.WSVToCSI2.EffectiveMass = effectiveMass2 * effectiveMassCFMScale;
            data.WSVToCSI2.AngularA = angularJacobians.AngularA2;
            data.WSVToCSI2.AngularB = angularJacobians.AngularB2;
            data.WSVToCSI3.EffectiveMass = effectiveMass3 * effectiveMassCFMScale;
            data.WSVToCSI3.AngularA = angularJacobians.AngularA3;
            data.WSVToCSI3.AngularB = angularJacobians.AngularB3;

            var biasVelocity0 = Vector.Min(error0 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            var biasVelocity1 = Vector.Min(error1 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            var biasVelocity2 = Vector.Min(error2 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            var biasVelocity3 = Vector.Min(error3 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            data.BiasImpulse0 = biasVelocity0 * data.WSVToCSI0.EffectiveMass;
            data.BiasImpulse1 = biasVelocity1 * data.WSVToCSI1.EffectiveMass;
            data.BiasImpulse2 = biasVelocity2 * data.WSVToCSI2.EffectiveMass;
            data.BiasImpulse3 = biasVelocity3 * data.WSVToCSI3.EffectiveMass;


        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        public static void ApplyImpulse(ref ContactPenetrationLimitCSIToWSV csiToWSV, ref Vector3Wide normal,
            ref Vector<float> inverseMassA, ref Vector<float> inverseMassB,
            ref Vector<float> correctiveImpulse,
            ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            BodyVelocities correctiveVelocityA, correctiveVelocityB;
            var linearVelocityChangeA = correctiveImpulse * inverseMassA;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeA, out correctiveVelocityA.LinearVelocity);
            Vector3Wide.Scale(ref csiToWSV.AngularA, ref correctiveImpulse, out correctiveVelocityA.AngularVelocity);
            var linearVelocityChangeB = correctiveImpulse * inverseMassB;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeB, out correctiveVelocityB.LinearVelocity);
            Vector3Wide.Scale(ref csiToWSV.AngularB, ref correctiveImpulse, out correctiveVelocityB.AngularVelocity);
            Vector3Wide.Add(ref wsvA.LinearVelocity, ref correctiveVelocityA.LinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref wsvA.AngularVelocity, ref correctiveVelocityA.AngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref correctiveVelocityB.LinearVelocity, out wsvB.LinearVelocity); //Note subtract; normal = -jacobianLinearB
            Vector3Wide.Add(ref wsvB.AngularVelocity, ref correctiveVelocityB.AngularVelocity, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            ref ContactPenetrationLimit4IterationData data,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2,
            ref Vector<float> accumulatedImpulse3, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ApplyImpulse(ref data.CSIToWSV0, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref accumulatedImpulse0, ref wsvA, ref wsvB);
            ApplyImpulse(ref data.CSIToWSV1, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref accumulatedImpulse1, ref wsvA, ref wsvB);
            ApplyImpulse(ref data.CSIToWSV2, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref accumulatedImpulse2, ref wsvA, ref wsvB);
            ApplyImpulse(ref data.CSIToWSV3, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref accumulatedImpulse3, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB,
            ref ContactPenetrationLimitWSVToCSI wsvToCSI,
            ref Vector3Wide normal, ref Vector<float> biasImpulse, ref Vector<float> softnessImpulseScale,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            //Note that we do NOT use pretransformed jacobians here; the linear jacobian sharing (normal) meant that we had the effective mass anyway.
            Vector3Wide.Dot(ref wsvA.LinearVelocity, ref normal, out var csvaLinear);
            Vector3Wide.Dot(ref wsvA.AngularVelocity, ref wsvToCSI.AngularA, out var csvaAngular);
            Vector3Wide.Dot(ref wsvB.LinearVelocity, ref normal, out var negatedCSVBLinear);
            Vector3Wide.Dot(ref wsvB.AngularVelocity, ref wsvToCSI.AngularB, out var csvbAngular);
            var csi = biasImpulse - accumulatedImpulse * softnessImpulseScale - (csvaLinear - negatedCSVBLinear + csvaAngular + csvbAngular) * wsvToCSI.EffectiveMass;

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse + csi);

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref ContactPenetrationLimit4IterationData data,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2,
            ref Vector<float> accumulatedImpulse3, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI0, ref data.Normal, ref data.BiasImpulse0, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse0, out var correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV0, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI1, ref data.Normal, ref data.BiasImpulse1, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse1, out correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV1, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI2, ref data.Normal, ref data.BiasImpulse2, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse2, out correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV2, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI3, ref data.Normal, ref data.BiasImpulse3, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse3, out correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV3, ref data.Normal, ref data.InverseMassA, ref data.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
