using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype
{

    public struct ContactPenetrationLimit4Jacobians
    {
        public Vector3Wide Normal;
        public Vector3Wide AngularA0;
        public Vector3Wide AngularB0;
        public Vector3Wide AngularA1;
        public Vector3Wide AngularB1;
        public Vector3Wide AngularA2;
        public Vector3Wide AngularB2;
        public Vector3Wide AngularA3;
        public Vector3Wide AngularB3;
    }


    public struct ContactPenetrationLimitTransform
    {
        public Vector3Wide LinearA;
        public Vector3Wide LinearB;
        public Vector3Wide AngularA;
        public Vector3Wide AngularB;
    }
    /// <summary>
    /// Four convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public struct ContactPenetrationLimit4IterationData
    {
        //TODO: Note that all linear components are simply the normal vector times the effective mass (negated for B).
        //Storing only the jacobians (normals) and then multipying by the effective mass (a scalar per contact)
        //would save us a good bit of space. Worth trying.
        public ContactPenetrationLimitTransform WSVToCSI0;
        public ContactPenetrationLimitTransform WSVToCSI1;
        public ContactPenetrationLimitTransform WSVToCSI2;
        public ContactPenetrationLimitTransform WSVToCSI3;

        public Vector<float> BiasImpulse0;
        public Vector<float> BiasImpulse1;
        public Vector<float> BiasImpulse2;
        public Vector<float> BiasImpulse3;
        //Recall that the softness impulse scale is independent of effective mass due to cancellation.
        public Vector<float> SoftnessImpulseScale;

        //Nothing above is accessed by the warm start. Should really try splitting it.


        //Again, note here that you could just store the shared linear jacobian (the contact manifold normal).
        //The M^-1 term would have to be reintroduced, but for linear terms it's only 4 scalars per lane, compared to 21 scalars for the redundant jacobians.

        public ContactPenetrationLimitTransform CSIToWSV0;
        public ContactPenetrationLimitTransform CSIToWSV1;
        public ContactPenetrationLimitTransform CSIToWSV2;
        public ContactPenetrationLimitTransform CSIToWSV3;

        //The first half would introduce 4 multiplies, and save 16 loads.
        //The second half would introduce 4 multiplies, and save 16 loads.
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
            jacobians.Normal = normal;
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
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref ContactPenetrationLimit4Jacobians jacobians, ref SpringSettings springSettings,
            ref Vector<float> error0,
            ref Vector<float> error1,
            ref Vector<float> error2,
            ref Vector<float> error3, float dt, float inverseDt, out ContactPenetrationLimit4IterationData data)
        {
            //effective mass
            //Note redundancy of linear components. Pretty silly to keep them around!
            Vector3Wide.Scale(ref jacobians.Normal, ref inertiaA.InverseMass, out data.CSIToWSV0.LinearA);
            Vector3Wide.Negate(ref data.CSIToWSV0.LinearA, out data.CSIToWSV0.LinearB);
            Vector3Wide.Dot(ref data.CSIToWSV0.LinearA, ref jacobians.Normal, out var linear);

            data.CSIToWSV1.LinearA = data.CSIToWSV0.LinearA;
            data.CSIToWSV2.LinearA = data.CSIToWSV0.LinearA;
            data.CSIToWSV3.LinearA = data.CSIToWSV0.LinearA;
            data.CSIToWSV1.LinearB = data.CSIToWSV0.LinearB;
            data.CSIToWSV2.LinearB = data.CSIToWSV0.LinearB;
            data.CSIToWSV3.LinearB = data.CSIToWSV0.LinearB;

            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularA0, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV0.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularB0, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV0.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV0.AngularA, ref jacobians.AngularA0, out var angularA0);
            Vector3Wide.Dot(ref data.CSIToWSV0.AngularB, ref jacobians.AngularB0, out var angularB0);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularA1, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV1.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularB1, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV1.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV1.AngularA, ref jacobians.AngularA1, out var angularA1);
            Vector3Wide.Dot(ref data.CSIToWSV1.AngularB, ref jacobians.AngularB1, out var angularB1);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularA2, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV2.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularB2, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV2.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV2.AngularA, ref jacobians.AngularA2, out var angularA2);
            Vector3Wide.Dot(ref data.CSIToWSV2.AngularB, ref jacobians.AngularB2, out var angularB2);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularA3, ref inertiaA.InverseInertiaTensor, out data.CSIToWSV3.AngularA);
            Matrix3x3Wide.TransformWithoutOverlap(ref jacobians.AngularB3, ref inertiaB.InverseInertiaTensor, out data.CSIToWSV3.AngularB);
            Vector3Wide.Dot(ref data.CSIToWSV3.AngularA, ref jacobians.AngularA3, out var angularA3);
            Vector3Wide.Dot(ref data.CSIToWSV3.AngularB, ref jacobians.AngularB3, out var angularB3);


            var twoLinear = linear + linear;
            var effectiveMass0 = Vector<float>.One / (twoLinear + angularA0 + angularB0);
            var effectiveMass1 = Vector<float>.One / (twoLinear + angularA1 + angularB1);
            var effectiveMass2 = Vector<float>.One / (twoLinear + angularA2 + angularB2);
            var effectiveMass3 = Vector<float>.One / (twoLinear + angularA3 + angularB3);

            var frequencyDt = springSettings.NaturalFrequency * dt;
            var twiceDampingRatio = springSettings.DampingRatio * 2; //Could precompute.
            var extra = Vector<float>.One / (frequencyDt * (frequencyDt + twiceDampingRatio));
            var effectiveMassCFMScale = Vector<float>.One / (Vector<float>.One + extra);
            var softenedEffectiveMass0 = effectiveMass0 * effectiveMassCFMScale;
            var softenedEffectiveMass1 = effectiveMass1 * effectiveMassCFMScale;
            var softenedEffectiveMass2 = effectiveMass2 * effectiveMassCFMScale;
            var softenedEffectiveMass3 = effectiveMass3 * effectiveMassCFMScale;

            data.SoftnessImpulseScale = extra * effectiveMassCFMScale;

            var positionErrorToVelocity = springSettings.NaturalFrequency / (frequencyDt + twiceDampingRatio);

            var biasVelocity0 = Vector.Min(error0 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            var biasVelocity1 = Vector.Min(error1 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            var biasVelocity2 = Vector.Min(error2 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            var biasVelocity3 = Vector.Min(error3 * positionErrorToVelocity, springSettings.MaximumRecoveryVelocity);
            data.BiasImpulse0 = biasVelocity0 * softenedEffectiveMass0;
            data.BiasImpulse1 = biasVelocity1 * softenedEffectiveMass1;
            data.BiasImpulse2 = biasVelocity2 * softenedEffectiveMass2;
            data.BiasImpulse3 = biasVelocity3 * softenedEffectiveMass3;

            //Precompute the wsv * (JT * softenedEffectiveMass) term.
            //Note redundancy again.
            Vector3Wide.Negate(ref jacobians.Normal, out var jLinearB);
            Vector3Wide.Scale(ref jacobians.Normal, ref softenedEffectiveMass0, out data.WSVToCSI0.LinearA);
            Vector3Wide.Scale(ref jLinearB, ref softenedEffectiveMass0, out data.WSVToCSI0.LinearB);
            Vector3Wide.Scale(ref jacobians.AngularA0, ref softenedEffectiveMass0, out data.WSVToCSI0.AngularA);
            Vector3Wide.Scale(ref jacobians.AngularB0, ref softenedEffectiveMass0, out data.WSVToCSI0.AngularB);

            Vector3Wide.Scale(ref jacobians.Normal, ref softenedEffectiveMass1, out data.WSVToCSI1.LinearA);
            Vector3Wide.Scale(ref jLinearB, ref softenedEffectiveMass1, out data.WSVToCSI1.LinearB);
            Vector3Wide.Scale(ref jacobians.AngularA1, ref softenedEffectiveMass1, out data.WSVToCSI1.AngularA);
            Vector3Wide.Scale(ref jacobians.AngularB1, ref softenedEffectiveMass1, out data.WSVToCSI1.AngularB);

            Vector3Wide.Scale(ref jacobians.Normal, ref softenedEffectiveMass2, out data.WSVToCSI2.LinearA);
            Vector3Wide.Scale(ref jLinearB, ref softenedEffectiveMass2, out data.WSVToCSI2.LinearB);
            Vector3Wide.Scale(ref jacobians.AngularA2, ref softenedEffectiveMass2, out data.WSVToCSI2.AngularA);
            Vector3Wide.Scale(ref jacobians.AngularB2, ref softenedEffectiveMass2, out data.WSVToCSI2.AngularB);

            Vector3Wide.Scale(ref jacobians.Normal, ref softenedEffectiveMass3, out data.WSVToCSI3.LinearA);
            Vector3Wide.Scale(ref jLinearB, ref softenedEffectiveMass3, out data.WSVToCSI3.LinearB);
            Vector3Wide.Scale(ref jacobians.AngularA3, ref softenedEffectiveMass3, out data.WSVToCSI3.AngularA);
            Vector3Wide.Scale(ref jacobians.AngularB3, ref softenedEffectiveMass3, out data.WSVToCSI3.AngularB);
        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        public static void ApplyImpulse(ref ContactPenetrationLimitTransform csiToWSV,
            ref Vector<float> correctiveImpulse,
            ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            BodyVelocities correctiveVelocityA, correctiveVelocityB;
            Vector3Wide.Scale(ref csiToWSV.LinearA, ref correctiveImpulse, out correctiveVelocityA.LinearVelocity);
            Vector3Wide.Scale(ref csiToWSV.AngularA, ref correctiveImpulse, out correctiveVelocityA.AngularVelocity);
            Vector3Wide.Scale(ref csiToWSV.LinearB, ref correctiveImpulse, out correctiveVelocityB.LinearVelocity);
            Vector3Wide.Scale(ref csiToWSV.AngularB, ref correctiveImpulse, out correctiveVelocityB.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.LinearVelocity, ref wsvA.LinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.AngularVelocity, ref wsvA.AngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.LinearVelocity, ref wsvB.LinearVelocity, out wsvB.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.AngularVelocity, ref wsvB.AngularVelocity, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            ref ContactPenetrationLimit4IterationData data,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2,
            ref Vector<float> accumulatedImpulse3, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ApplyImpulse(ref data.CSIToWSV0, ref accumulatedImpulse0, ref wsvA, ref wsvB);
            ApplyImpulse(ref data.CSIToWSV1, ref accumulatedImpulse1, ref wsvA, ref wsvB);
            ApplyImpulse(ref data.CSIToWSV2, ref accumulatedImpulse2, ref wsvA, ref wsvB);
            ApplyImpulse(ref data.CSIToWSV3, ref accumulatedImpulse3, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB,
            ref ContactPenetrationLimitTransform wsvToCSI, ref Vector<float> biasImpulse, ref Vector<float> softnessImpulseScale, 
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            //Take the world space velocity of each body into constraint space by transforming by the transpose(jacobian).
            //(The jacobian is a row vector by convention, while we treat our velocity vectors as a 12x1 row vector for the purposes of constraint space velocity calculation.
            //So we are multiplying v * JT.)
            //Then, transform it into an impulse by applying the effective mass.
            //Here, we combine the projection and impulse conversion into a precomputed value, i.e. v * (JT * softenedEffectiveMass).
            Vector3Wide.Dot(ref wsvA.LinearVelocity, ref wsvToCSI.LinearA, out var csiaLinear);
            Vector3Wide.Dot(ref wsvA.AngularVelocity, ref wsvToCSI.AngularA, out var csiaAngular);
            Vector3Wide.Dot(ref wsvB.LinearVelocity, ref wsvToCSI.LinearB, out var csibLinear);
            Vector3Wide.Dot(ref wsvB.AngularVelocity, ref wsvToCSI.AngularB, out var csibAngular);
            var csi = biasImpulse - accumulatedImpulse * softnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);

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
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI0, ref data.BiasImpulse0, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse0, out var correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV0, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI1, ref data.BiasImpulse1, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse1, out correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV1, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI2, ref data.BiasImpulse2, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse2, out correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV2, ref correctiveCSI, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data.WSVToCSI3, ref data.BiasImpulse3, ref data.SoftnessImpulseScale,
                ref accumulatedImpulse3, out correctiveCSI);
            ApplyImpulse(ref data.CSIToWSV3, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
