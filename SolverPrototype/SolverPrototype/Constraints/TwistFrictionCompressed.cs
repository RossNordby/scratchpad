﻿using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{
    //For in depth explanations of constraints, check the Inequality1DOF.cs implementation.
    //The details are omitted for brevity in other implementations.

    public struct TwistFrictionProjectionCompressed
    {
        //Jacobians and inertia are shared with other constraints.
        public Vector<float> EffectiveMass;
    }

    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TwistFrictionCompressed
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide angularJacobianA,
            out TwistFrictionProjectionCompressed projection)
        {
            //Compute effective mass matrix contributions. No linear contributions for the twist constraint.
            //Note that we use the angularJacobianA (that is, the normal) for both, despite angularJacobianB = -angularJacobianA. That's fine- J * M * JT is going to be positive regardless.
            Matrix3x3Wide.Transform(ref angularJacobianA, ref inertiaA.InverseInertiaTensor, out var aIntermediate);
            Matrix3x3Wide.Transform(ref angularJacobianA, ref inertiaB.InverseInertiaTensor, out var bIntermediate);
            Vector3Wide.Dot(ref aIntermediate, ref angularJacobianA, out var angularA);
            Vector3Wide.Dot(ref bIntermediate, ref angularJacobianA, out var angularB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            //Note that we have to guard against two bodies with infinite inertias. This is a valid state! 
            //(We do not have to do such guarding on constraints with linear jacobians; dynamic bodies cannot have zero *mass*.)
            //(Also note that there's no need for epsilons here... users shouldn't be setting their inertias to the absurd values it would take to cause a problem.
            //Invalid conditions can't arise dynamically.)
            var inverseEffectiveMass = angularA + angularB;
            var inverseIsZero = Vector.Equals(Vector<float>.Zero, inverseEffectiveMass);
            projection.EffectiveMass = Vector.ConditionalSelect(inverseIsZero, Vector<float>.Zero, Vector<float>.One / (angularA + angularB));

            //Note that friction constraints have no bias velocity. They target zero velocity.


        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Vector3Wide angularJacobianA, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref Vector<float> correctiveImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            Vector3Wide.Scale(ref angularJacobianA, ref correctiveImpulse, out var worldCorrectiveImpulseA);
            Matrix3x3Wide.TransformWithoutOverlap(ref worldCorrectiveImpulseA, ref inertiaA.InverseInertiaTensor, out var worldCorrectiveVelocityA);
            Matrix3x3Wide.TransformWithoutOverlap(ref worldCorrectiveImpulseA, ref inertiaB.InverseInertiaTensor, out var worldCorrectiveVelocityB);
            Vector3Wide.Add(ref wsvA.AngularVelocity, ref worldCorrectiveVelocityA, out wsvA.AngularVelocity);
            Vector3Wide.Subtract(ref wsvB.AngularVelocity, ref worldCorrectiveVelocityB, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref Vector3Wide angularJacobianA, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ApplyImpulse(ref angularJacobianA, ref inertiaA, ref inertiaB, ref accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref Vector3Wide angularJacobianA, ref TwistFrictionProjectionCompressed projection, 
            ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref Vector<float> maximumImpulse,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            Vector3Wide.Dot(ref wsvA.AngularVelocity, ref angularJacobianA, out var csvA);
            Vector3Wide.Dot(ref wsvB.AngularVelocity, ref angularJacobianA, out var negatedCSVB);
            var negativeCSI = (csvA + negatedCSVB) * projection.EffectiveMass; //Since there is no bias or softness to give us the negative, we just do it when we apply to the accumulated impulse.

            var previousAccumulated = accumulatedImpulse;
            //The maximum force of friction depends upon the normal impulse.
            accumulatedImpulse = Vector.Min(maximumImpulse, Vector.Max(-maximumImpulse, accumulatedImpulse - negativeCSI));

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref Vector3Wide angularJacobianA, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref TwistFrictionProjectionCompressed projection, 
            ref Vector<float> maximumImpulse, ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref angularJacobianA, ref projection, ref wsvA, ref wsvB, ref maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref angularJacobianA, ref inertiaA, ref inertiaB, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
