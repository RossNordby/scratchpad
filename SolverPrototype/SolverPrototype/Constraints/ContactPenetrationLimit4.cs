﻿using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SolverPrototype.Constraints
{

    public struct ContactPenetrationLimitProjection
    {
        //Note that these are just the raw jacobians, no precomputation with the JT*EffectiveMass.
        public Vector3Wide AngularA;
        public Vector3Wide AngularB;
        public Vector<float> EffectiveMass;
        public Vector<float> BiasVelocity;
    }

    /// <summary>
    /// Data required to project world space velocities into a constraint impulse.
    /// </summary>
    public struct ContactPenetrationLimit4Projection
    {
        //Note that the data is interleaved to match the access order. We solve each constraint one at a time internally.
        //Also, the normal and inertias are shared across all constraints.
        public ContactPenetrationLimitProjection Penetration0;
        public Vector<float> SoftnessImpulseScale;
        public ContactPenetrationLimitProjection Penetration1;
        public ContactPenetrationLimitProjection Penetration2;
        public ContactPenetrationLimitProjection Penetration3;
    }


    /// <summary>
    /// Four convex-sourced contact penetration limits solved together. Internally implemented using SI solver. 
    /// Batching saves on redundant data.
    /// </summary>
    public static class ContactPenetrationLimit4
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal, ref ContactManifold4PrestepData prestep, float dt, float inverseDt,
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
            Vector3Wide.CrossWithoutOverlap(ref prestep.OffsetA0, ref normal, out projection.Penetration0.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref prestep.OffsetB0, out projection.Penetration0.AngularB);
            Vector3Wide.CrossWithoutOverlap(ref prestep.OffsetA1, ref normal, out projection.Penetration1.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref prestep.OffsetB1, out projection.Penetration1.AngularB);
            Vector3Wide.CrossWithoutOverlap(ref prestep.OffsetA2, ref normal, out projection.Penetration2.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref prestep.OffsetB2, out projection.Penetration2.AngularB);
            Vector3Wide.CrossWithoutOverlap(ref prestep.OffsetA3, ref normal, out projection.Penetration3.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref normal, ref prestep.OffsetB3, out projection.Penetration3.AngularB);

            //effective mass
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration0.AngularA, ref inertiaA.InverseInertiaTensor, out var intermediateA0);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration0.AngularB, ref inertiaB.InverseInertiaTensor, out var intermediateB0);
            Vector3Wide.Dot(ref intermediateA0, ref projection.Penetration0.AngularA, out var angularA0);
            Vector3Wide.Dot(ref intermediateB0, ref projection.Penetration0.AngularB, out var angularB0);

            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration1.AngularA, ref inertiaA.InverseInertiaTensor, out var intermediateA1);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration1.AngularB, ref inertiaB.InverseInertiaTensor, out var intermediateB1);
            Vector3Wide.Dot(ref intermediateA1, ref projection.Penetration1.AngularA, out var angularA1);
            Vector3Wide.Dot(ref intermediateB1, ref projection.Penetration1.AngularB, out var angularB1);

            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration2.AngularA, ref inertiaA.InverseInertiaTensor, out var intermediateA2);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration2.AngularB, ref inertiaB.InverseInertiaTensor, out var intermediateB2);
            Vector3Wide.Dot(ref intermediateA2, ref projection.Penetration2.AngularA, out var angularA2);
            Vector3Wide.Dot(ref intermediateB2, ref projection.Penetration2.AngularB, out var angularB2);

            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration3.AngularA, ref inertiaA.InverseInertiaTensor, out var intermediateA3);
            Matrix3x3Wide.TransformWithoutOverlap(ref projection.Penetration3.AngularB, ref inertiaB.InverseInertiaTensor, out var intermediateB3);
            Vector3Wide.Dot(ref intermediateA3, ref projection.Penetration3.AngularA, out var angularA3);
            Vector3Wide.Dot(ref intermediateB3, ref projection.Penetration3.AngularB, out var angularB3);

            //Linear effective mass contribution notes:
            //1) The J * M^-1 * JT can be reordered to J * JT * M^-1 for the linear components, since M^-1 is a scalar and dot(n * scalar, n) = dot(n, n) * scalar.
            //2) dot(normal, normal) == 1, so the contribution from each body is just its inverse mass.
            Springiness.ComputeSpringiness(ref prestep.SpringSettings, dt, 4f, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var linear = inertiaA.InverseMass + inertiaB.InverseMass;
            //Note that we don't precompute the JT * effectiveMass term. Since the jacobians are shared, we have to do that multiply anyway.
            projection.Penetration0.EffectiveMass = effectiveMassCFMScale / (linear + angularA0 + angularB0);
            projection.Penetration1.EffectiveMass = effectiveMassCFMScale / (linear + angularA1 + angularB1);
            projection.Penetration2.EffectiveMass = effectiveMassCFMScale / (linear + angularA2 + angularB2);
            projection.Penetration3.EffectiveMass = effectiveMassCFMScale / (linear + angularA3 + angularB3);            

            projection.Penetration0.BiasVelocity = Vector.Min(prestep.PenetrationDepth0 * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
            projection.Penetration1.BiasVelocity = Vector.Min(prestep.PenetrationDepth1 * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
            projection.Penetration2.BiasVelocity = Vector.Min(prestep.PenetrationDepth2 * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
            projection.Penetration3.BiasVelocity = Vector.Min(prestep.PenetrationDepth3 * positionErrorToVelocity, prestep.SpringSettings.MaximumRecoveryVelocity);
        }


        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref ContactPenetrationLimitProjection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> correctiveImpulse,
            ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            var linearVelocityChangeA = correctiveImpulse * inertiaA.InverseMass;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeA, out var correctiveVelocityALinearVelocity);
            Vector3Wide.Scale(ref projection.AngularA, ref correctiveImpulse, out var correctiveAngularImpulseA);
            Matrix3x3Wide.TransformWithoutOverlap(ref correctiveAngularImpulseA, ref inertiaA.InverseInertiaTensor, out var correctiveVelocityAAngularVelocity);

            var linearVelocityChangeB = correctiveImpulse * inertiaB.InverseMass;
            Vector3Wide.Scale(ref normal, ref linearVelocityChangeB, out var correctiveVelocityBLinearVelocity);
            Vector3Wide.Scale(ref projection.AngularB, ref correctiveImpulse, out var correctiveAngularImpulseB);
            Matrix3x3Wide.TransformWithoutOverlap(ref correctiveAngularImpulseB, ref inertiaB.InverseInertiaTensor, out var correctiveVelocityBAngularVelocity);

            Vector3Wide.Add(ref wsvA.LinearVelocity, ref correctiveVelocityALinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref wsvA.AngularVelocity, ref correctiveVelocityAAngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref correctiveVelocityBLinearVelocity, out wsvB.LinearVelocity); //Note subtract; normal = -jacobianLinearB
            Vector3Wide.Add(ref wsvB.AngularVelocity, ref correctiveVelocityBAngularVelocity, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(
            ref ContactPenetrationLimit4Projection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2,
            ref Vector<float> accumulatedImpulse3, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse0, ref wsvA, ref wsvB);
            ApplyImpulse(ref projection.Penetration1, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse1, ref wsvA, ref wsvB);
            ApplyImpulse(ref projection.Penetration2, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse2, ref wsvA, ref wsvB);
            ApplyImpulse(ref projection.Penetration3, ref inertiaA, ref inertiaB, ref normal, ref accumulatedImpulse3, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB,
            ref ContactPenetrationLimitProjection projection,
            ref Vector3Wide normal, ref Vector<float> softnessImpulseScale,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            //Note that we do NOT use pretransformed jacobians here; the linear jacobian sharing (normal) meant that we had the effective mass anyway.
            Vector3Wide.Dot(ref wsvA.LinearVelocity, ref normal, out var csvaLinear);
            Vector3Wide.Dot(ref wsvA.AngularVelocity, ref projection.AngularA, out var csvaAngular);
            Vector3Wide.Dot(ref wsvB.LinearVelocity, ref normal, out var negatedCSVBLinear);
            Vector3Wide.Dot(ref wsvB.AngularVelocity, ref projection.AngularB, out var csvbAngular);
            //Compute negated version to avoid the need for an explicit negate.
            var negatedCSI = accumulatedImpulse * softnessImpulseScale + (csvaLinear - negatedCSVBLinear + csvaAngular + csvbAngular - projection.BiasVelocity) * projection.EffectiveMass;

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse - negatedCSI);

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref ContactPenetrationLimit4Projection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide normal,
            ref Vector<float> accumulatedImpulse0,
            ref Vector<float> accumulatedImpulse1,
            ref Vector<float> accumulatedImpulse2,
            ref Vector<float> accumulatedImpulse3, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration0, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse0, out var correctiveCSI0);
            ApplyImpulse(ref projection.Penetration0, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI0, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration1, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse1, out var correctiveCSI1);
            ApplyImpulse(ref projection.Penetration1, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI1, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration2, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse2, out var correctiveCSI2);
            ApplyImpulse(ref projection.Penetration2, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI2, ref wsvA, ref wsvB);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection.Penetration3, ref normal, ref projection.SoftnessImpulseScale, ref accumulatedImpulse3, out var correctiveCSI3);
            ApplyImpulse(ref projection.Penetration3, ref inertiaA, ref inertiaB, ref normal, ref correctiveCSI3, ref wsvA, ref wsvB);
        }

    }
}
