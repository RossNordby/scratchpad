using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype
{

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
    /// Data required to project world space velocities into a constraint impulse.
    /// </summary>
    public struct ContactPenetrationLimit4Projection
    {
        //Note that all linear components of WSVToCSI are simply the normal vector times the effective mass (negated for B).
        //Storing only the linear jacobian (normal) and then multiplying by the effective mass (a scalar per contact) saves a bunch of stores and loads.
        //Note that the projection and unprojection data is interleaved to match the access order. We solve each constraint one at a time internally.
        public Vector3Wide Normal;
        public ContactPenetrationLimitWSVToCSI WSVToCSI0;
        public Vector<float> BiasImpulse0;
        public Vector<float> SoftnessImpulseScale;
        public Vector<float> InverseMassA;
        public Vector<float> InverseMassB;
        public ContactPenetrationLimitCSIToWSV CSIToWSV0;
        public ContactPenetrationLimitWSVToCSI WSVToCSI1;
        public Vector<float> BiasImpulse1;
        public ContactPenetrationLimitCSIToWSV CSIToWSV1;
        public ContactPenetrationLimitWSVToCSI WSVToCSI2;
        public Vector<float> BiasImpulse2;
        public ContactPenetrationLimitCSIToWSV CSIToWSV2;
        public ContactPenetrationLimitWSVToCSI WSVToCSI3;
        public Vector<float> BiasImpulse3;
        public ContactPenetrationLimitCSIToWSV CSIToWSV3;
    }


    /// <summary>
    /// Four convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public static class ContactPenetrationLimit4
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref ContactManifold4PrestepData prestep, float dt, float inverseDt,
            out ContactPenetrationLimit4Projection projection)
        {
            //We directly take the prestep data here since the jacobians and error don't undergo any processing.

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
            //Note that we leave the penetration depth as is, even when it's negative. Speculative contacts!
            Vector3Wide.CrossWithoutOverlap(ref prestep.Contact0.OffsetA, ref prestep.Normal, out projection.WSVToCSI0.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref prestep.Normal, ref prestep.Contact0.OffsetB, out projection.WSVToCSI0.AngularB);
            Vector3Wide.CrossWithoutOverlap(ref prestep.Contact1.OffsetA, ref prestep.Normal, out projection.WSVToCSI1.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref prestep.Normal, ref prestep.Contact1.OffsetB, out projection.WSVToCSI1.AngularB);
            Vector3Wide.CrossWithoutOverlap(ref prestep.Contact2.OffsetA, ref prestep.Normal, out projection.WSVToCSI2.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref prestep.Normal, ref prestep.Contact2.OffsetB, out projection.WSVToCSI2.AngularB);
            Vector3Wide.CrossWithoutOverlap(ref prestep.Contact3.OffsetA, ref prestep.Normal, out projection.WSVToCSI3.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref prestep.Normal, ref prestep.Contact3.OffsetB, out projection.WSVToCSI3.AngularB);

            //effective mass
            //Note that the linear components are all redundant due to the shared normal.
            //Also note that the J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since each constraint is 1DOF (scalar multiply commutativity).

            projection.Normal = prestep.Normal;
            Vector3Wide.Dot(ref prestep.Normal, ref prestep.Normal, out var normalLengthSquared);
            var linearA = normalLengthSquared * inertiaA.InverseMass;
            var linearB = normalLengthSquared * inertiaB.InverseMass;
            var linear = linearA + linearB;

            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI0.AngularA, ref inertiaA.InverseInertiaTensor, out projection.CSIToWSV0.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI0.AngularB, ref inertiaB.InverseInertiaTensor, out projection.CSIToWSV0.AngularB);
            Vector3Wide.Dot(ref projection.CSIToWSV0.AngularA, ref projection.WSVToCSI0.AngularA, out var angularA0);
            Vector3Wide.Dot(ref projection.CSIToWSV0.AngularB, ref projection.WSVToCSI0.AngularB, out var angularB0);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI1.AngularA, ref inertiaA.InverseInertiaTensor, out projection.CSIToWSV1.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI1.AngularB, ref inertiaB.InverseInertiaTensor, out projection.CSIToWSV1.AngularB);
            Vector3Wide.Dot(ref projection.CSIToWSV1.AngularA, ref projection.WSVToCSI1.AngularA, out var angularA1);
            Vector3Wide.Dot(ref projection.CSIToWSV1.AngularB, ref projection.WSVToCSI1.AngularB, out var angularB1);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI2.AngularA, ref inertiaA.InverseInertiaTensor, out projection.CSIToWSV2.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI2.AngularB, ref inertiaB.InverseInertiaTensor, out projection.CSIToWSV2.AngularB);
            Vector3Wide.Dot(ref projection.CSIToWSV2.AngularA, ref projection.WSVToCSI2.AngularA, out var angularA2);
            Vector3Wide.Dot(ref projection.CSIToWSV2.AngularB, ref projection.WSVToCSI2.AngularB, out var angularB2);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI3.AngularA, ref inertiaA.InverseInertiaTensor, out projection.CSIToWSV3.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.WSVToCSI3.AngularB, ref inertiaB.InverseInertiaTensor, out projection.CSIToWSV3.AngularB);
            Vector3Wide.Dot(ref projection.CSIToWSV3.AngularA, ref projection.WSVToCSI3.AngularA, out var angularA3);
            Vector3Wide.Dot(ref projection.CSIToWSV3.AngularB, ref projection.WSVToCSI3.AngularB, out var angularB3);
            projection.InverseMassA = inertiaA.InverseMass;
            projection.InverseMassB = inertiaB.InverseMass;

            var effectiveMass0 = Vector<float>.One / (linear + angularA0 + angularB0);
            var effectiveMass1 = Vector<float>.One / (linear + angularA1 + angularB1);
            var effectiveMass2 = Vector<float>.One / (linear + angularA2 + angularB2);
            var effectiveMass3 = Vector<float>.One / (linear + angularA3 + angularB3);
            
            Springiness.ComputeSpringiness(ref prestep.SpringSettings, dt, 4f, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);

            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.WSVToCSI0.EffectiveMass = effectiveMass0 * effectiveMassCFMScale;
            projection.WSVToCSI1.EffectiveMass = effectiveMass1 * effectiveMassCFMScale;
            projection.WSVToCSI2.EffectiveMass = effectiveMass2 * effectiveMassCFMScale;
            projection.WSVToCSI3.EffectiveMass = effectiveMass3 * effectiveMassCFMScale;

            var biasVelocity0 = Vector.Min(prestep.Contact0.PenetrationDepth * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
            var biasVelocity1 = Vector.Min(prestep.Contact1.PenetrationDepth * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
            var biasVelocity2 = Vector.Min(prestep.Contact2.PenetrationDepth * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
            var biasVelocity3 = Vector.Min(prestep.Contact3.PenetrationDepth * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
            projection.BiasImpulse0 = biasVelocity0 * projection.WSVToCSI0.EffectiveMass;
            projection.BiasImpulse1 = biasVelocity1 * projection.WSVToCSI1.EffectiveMass;
            projection.BiasImpulse2 = biasVelocity2 * projection.WSVToCSI2.EffectiveMass;
            projection.BiasImpulse3 = biasVelocity3 * projection.WSVToCSI3.EffectiveMass;
        }


        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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
            ref ContactPenetrationLimit4Projection data,
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
        public static void Solve(ref ContactPenetrationLimit4Projection projection,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2,
            ref Vector<float> accumulatedImpulse3, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.WSVToCSI0, ref projection.Normal, ref projection.BiasImpulse0, ref projection.SoftnessImpulseScale,
                ref accumulatedImpulse0, out var correctiveCSI);
            ApplyImpulse(ref projection.CSIToWSV0, ref projection.Normal, ref projection.InverseMassA, ref projection.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.WSVToCSI1, ref projection.Normal, ref projection.BiasImpulse1, ref projection.SoftnessImpulseScale,
                ref accumulatedImpulse1, out correctiveCSI);
            ApplyImpulse(ref projection.CSIToWSV1, ref projection.Normal, ref projection.InverseMassA, ref projection.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.WSVToCSI2, ref projection.Normal, ref projection.BiasImpulse2, ref projection.SoftnessImpulseScale,
                ref accumulatedImpulse2, out correctiveCSI);
            ApplyImpulse(ref projection.CSIToWSV2, ref projection.Normal, ref projection.InverseMassA, ref projection.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.WSVToCSI3, ref projection.Normal, ref projection.BiasImpulse3, ref projection.SoftnessImpulseScale,
                ref accumulatedImpulse3, out correctiveCSI);
            ApplyImpulse(ref projection.CSIToWSV3, ref projection.Normal, ref projection.InverseMassA, ref projection.InverseMassB, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
