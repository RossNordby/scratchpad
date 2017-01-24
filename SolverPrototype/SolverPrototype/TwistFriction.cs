using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    //For in depth explanations of constraints, check the Inequality1DOF.cs implementation.
    //The details are omitted for brevity in other implementations.

    public struct TwistFrictionProjection
    {
        public Vector3Wide WSVtoCSIAngularA;
        public Vector3Wide WSVtoCSIAngularB;
        public Vector3Wide CSIToWSVAngularA;
        public Vector3Wide CSIToWSVAngularB;
    }    

    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TwistFriction
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector3Wide angularJacobianA,
            out TwistFrictionProjection projection)
        {

            //Compute effective mass matrix contributions. No linear contributions for the twist constraint.
            Matrix3x3Wide.Transform(ref angularJacobianA, ref inertiaA.InverseInertiaTensor, out projection.CSIToWSVAngularA);
            Vector3Wide.Negate(ref angularJacobianA, out var angularJacobianB);
            Matrix3x3Wide.Transform(ref angularJacobianB, ref inertiaB.InverseInertiaTensor, out projection.CSIToWSVAngularB);
            Vector3Wide.Dot(ref projection.CSIToWSVAngularA, ref angularJacobianA, out var angularA);
            Vector3Wide.Dot(ref projection.CSIToWSVAngularB, ref angularJacobianB, out var angularB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            //Note that we have to guard against two bodies with infinite inertias. This is a valid state! 
            //(We do not have to do such guarding on constraints with linear jacobians; dynamic bodies cannot have zero *mass*.)
            //(Also note that there's no need for epsilons here... users shouldn't be setting their inertias to the absurd values it would take to cause a problem.
            //Invalid conditions can't arise dynamically.)
            var inverseEffectiveMass = angularA + angularB;
            var inverseIsZero = Vector.Equals(Vector<float>.Zero, inverseEffectiveMass);
            var effectiveMass = Vector.ConditionalSelect(inverseIsZero, Vector<float>.Zero, Vector<float>.One / (angularA + angularB));
                
            //Note that friction constraints have no bias velocity. They target zero velocity.

            //Finally, compute the (transposed) transform for constraint space impulse to world space.
            Vector3Wide.Scale(ref angularJacobianA, ref effectiveMass, out projection.WSVtoCSIAngularA);
            Vector3Wide.Scale(ref angularJacobianB, ref effectiveMass, out projection.WSVtoCSIAngularB);

        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        public static void ApplyImpulse(ref TwistFrictionProjection data, ref Vector<float> correctiveImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            Vector3Wide.Scale(ref data.CSIToWSVAngularA, ref correctiveImpulse, out var correctiveVelocityA);
            Vector3Wide.Scale(ref data.CSIToWSVAngularB, ref correctiveImpulse, out var correctiveVelocityB);
            Vector3Wide.Add(ref correctiveVelocityA, ref wsvA.AngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityB, ref wsvB.AngularVelocity, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref TwistFrictionProjection data, ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(ref data, ref accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref TwistFrictionProjection data, ref Vector<float> maximumImpulse,
            ref Vector<float> accumulatedImpulse, out Vector<float> correctiveCSI)
        {
            Vector3Wide.Dot(ref wsvA.AngularVelocity, ref data.WSVtoCSIAngularA, out var csia);
            Vector3Wide.Dot(ref wsvB.AngularVelocity, ref data.WSVtoCSIAngularB, out var csib);
            var negativeCSI = csia + csib; //Since there is no bias or softness to give us the negative, we just do it when we apply to the accumulated impulse.

            var previousAccumulated = accumulatedImpulse;
            //The maximum force of friction depends upon the normal impulse.
            accumulatedImpulse = Vector.Min(maximumImpulse, Vector.Max(-maximumImpulse, accumulatedImpulse - negativeCSI));

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref TwistFrictionProjection projection, ref Vector<float> maximumImpulse, ref Vector<float> accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection, ref maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref projection, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
