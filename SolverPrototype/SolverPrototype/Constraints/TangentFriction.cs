using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{
    //For in depth explanations of constraints, check the Inequality1DOF.cs implementation.
    //The details are omitted for brevity in other implementations.

    public struct TangentFrictionProjection
    {
        //Jacobians are generated on the fly from the tangents and offsets.
        //The tangents are reconstructed from the surface basis.
        //This saves 11 floats per constraint relative to the seminaive baseline of two shared linear jacobians and four angular jacobians. 
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Matrix2x2Wide EffectiveMass;
    }

    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TangentFriction
    {
        public struct Jacobians
        {
            public Matrix2x3Wide LinearA;
            public Matrix2x3Wide AngularA;
            public Matrix2x3Wide AngularB;
        }
        //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref Vector3Wide offsetB,
            out Jacobians jacobians)
        {
            //Two velocity constraints: 
            //dot(velocity(p, A), tangentX) = dot(velocity(p, B), tangentX)
            //dot(velocity(p, A), tangentY) = dot(velocity(p, B), tangentY)
            //where velocity(p, A) is the velocity of a point p attached to object A.
            //velocity(p, A) = linearVelocityA + angularVelocityA x (p - positionA) = linearVelocityA + angularVelocityA x offsetA
            //so:
            //dot(velocity(p, A), tangentX) = dot(linearVelocityA, tangentX) + dot(angularVelocityA x offsetA, tangentX)
            //dot(velocity(p, A), tangentX) = dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA)
            //Restating the two constraints:
            //dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA) = dot(linearVelocityB, tangentX) + dot(offsetB x tangentX, angularVelocityB)
            //dot(linearVelocityA, tangentY) + dot(offsetA x tangentY, angularVelocityA) = dot(linearVelocityB, tangentY) + dot(offsetB x tangentY, angularVelocityB)
            //dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA) - dot(linearVelocityB, tangentX) - dot(offsetB x tangentX, angularVelocityB) = 0
            //dot(linearVelocityA, tangentY) + dot(offsetA x tangentY, angularVelocityA) - dot(linearVelocityB, tangentY) - dot(offsetB x tangentY, angularVelocityB) = 0

            //Since there are two constraints (2DOFs), there are two rows in the jacobian, which based on the above is:
            //jLinearA = [ tangentX ]
            //           [ tangentY ]
            //jAngularA = [ offsetA x tangentX ]
            //            [ offsetA x tangentY ]
            //jLinearB = [ -tangentX ]
            //           [ -tangentY ]
            //jAngularB = [ -offsetB x tangentX ] = [ tangentX x offsetB ]
            //            [ -offsetB x tangentY ]   [ tangentY x offsetB ]
            jacobians.LinearA.X = tangentX;
            jacobians.LinearA.Y = tangentY;
            Vector3Wide.CrossWithoutOverlap(ref offsetA, ref tangentX, out jacobians.AngularA.X);
            Vector3Wide.CrossWithoutOverlap(ref offsetA, ref jacobians.LinearA.Y, out jacobians.AngularA.Y);
            Vector3Wide.CrossWithoutOverlap(ref tangentX, ref offsetB, out jacobians.AngularB.X);
            Vector3Wide.CrossWithoutOverlap(ref jacobians.LinearA.Y, ref offsetB, out jacobians.AngularB.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref Vector3Wide offsetB,
            ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            out TangentFrictionProjection projection)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref offsetA, ref offsetB, out var jacobians);
            //Compute effective mass matrix contributions.
            Matrix2x3Wide.Scale(ref jacobians.LinearA, ref inertiaA.InverseMass, out var linearIntermediateA);
            Matrix2x3Wide.Scale(ref jacobians.LinearA, ref inertiaB.InverseMass, out var linearIntermediateB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref linearIntermediateA, ref jacobians.LinearA, out var linearContributionA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref linearIntermediateB, ref jacobians.LinearA, out var linearContributionB);

            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularA, ref inertiaA.InverseInertiaTensor, out var angularIntermediateA);
            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularB, ref inertiaB.InverseInertiaTensor, out var angularIntermediateB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref angularIntermediateA, ref jacobians.AngularA, out var angularContributionA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref angularIntermediateB, ref jacobians.AngularB, out var angularContributionB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Matrix2x2Wide.Add(ref linearContributionA, ref linearContributionB, out var linear);
            Matrix2x2Wide.Subtract(ref angularContributionA, ref angularContributionB, out var angular);
            Matrix2x2Wide.Add(ref linear, ref angular, out var inverseEffectiveMass);
            Matrix2x2Wide.InvertWithoutOverlap(ref inverseEffectiveMass, out projection.EffectiveMass);
            projection.OffsetA = offsetA;
            projection.OffsetB = offsetB;

            //Note that friction constraints have no bias velocity. They target zero velocity.
        }
        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Jacobians jacobians, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref Vector2Wide correctiveImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.LinearA, out var linearImpulseA);
            Vector3Wide.Scale(ref linearImpulseA, ref inertiaA.InverseMass, out var correctiveLinearVelocityA);

            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.AngularA, out var angularImpulseA);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularImpulseA, ref inertiaA.InverseInertiaTensor, out var correctiveAngularVelocityA);
            Vector3Wide.Scale(ref linearImpulseA, ref inertiaB.InverseMass, out var correctiveLinearVelocityB);

            Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.AngularB, out var angularImpulseB);
            Matrix3x3Wide.TransformWithoutOverlap(ref angularImpulseB, ref inertiaB.InverseInertiaTensor, out var correctiveAngularVelocityB);
            Vector3Wide.Add(ref wsvA.LinearVelocity, ref correctiveLinearVelocityA, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref wsvA.AngularVelocity, ref correctiveAngularVelocityA, out wsvA.AngularVelocity);
            Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref correctiveLinearVelocityB, out wsvB.LinearVelocity); //note subtract- we based it on the LinearA jacobian.
            Vector3Wide.Add(ref wsvB.AngularVelocity, ref correctiveAngularVelocityB, out wsvB.AngularVelocity);
        }
        ///// <summary>
        ///// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        ///// </summary>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static void ApplyImpulse(ref Jacobians jacobians, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
        //    ref Vector2Wide correctiveImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        //{
        //    Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.LinearA, out var linearImpulseA);
        //    Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.AngularA, out var angularImpulseA);
        //    Matrix2x3Wide.Transform(ref correctiveImpulse, ref jacobians.AngularB, out var angularImpulseB);
        //    BodyVelocities correctiveVelocityA, correctiveVelocityB;
        //    Vector3Wide.Scale(ref linearImpulseA, ref inertiaA.InverseMass, out correctiveVelocityA.LinearVelocity);
        //    Matrix3x3Wide.TransformWithoutOverlap(ref angularImpulseA, ref inertiaA.InverseInertiaTensor, out correctiveVelocityA.AngularVelocity);
        //    Vector3Wide.Scale(ref linearImpulseA, ref inertiaB.InverseMass, out correctiveVelocityB.LinearVelocity);
        //    Matrix3x3Wide.TransformWithoutOverlap(ref angularImpulseB, ref inertiaB.InverseInertiaTensor, out correctiveVelocityB.AngularVelocity);
        //    Vector3Wide.Add(ref wsvA.LinearVelocity, ref correctiveVelocityA.LinearVelocity, out wsvA.LinearVelocity);
        //    Vector3Wide.Add(ref wsvA.AngularVelocity, ref correctiveVelocityA.AngularVelocity, out wsvA.AngularVelocity);
        //    Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref correctiveVelocityB.LinearVelocity, out wsvB.LinearVelocity); //note subtract- we based it on the LinearA jacobian.
        //    Vector3Wide.Add(ref wsvB.AngularVelocity, ref correctiveVelocityB.AngularVelocity, out wsvB.AngularVelocity);
        //}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref TangentFrictionProjection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref projection.OffsetA, ref projection.OffsetB, out var jacobians);
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(ref jacobians, ref inertiaA, ref inertiaB, ref accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref TangentFrictionProjection data, ref Jacobians jacobians,
            ref Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, out Vector2Wide correctiveCSI)
        {
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.LinearVelocity, ref jacobians.LinearA, out var csvaLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.AngularVelocity, ref jacobians.AngularA, out var csvaAngular);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvB.LinearVelocity, ref jacobians.LinearA, out var csvbLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvB.AngularVelocity, ref jacobians.AngularB, out var csvbAngular);
            Vector2Wide.Subtract(ref csvaLinear, ref csvbLinear, out var csvLinear); //Subtract since we shared linearA for both.
            Vector2Wide.Add(ref csvaAngular, ref csvbAngular, out var csvAngular);
            Vector2Wide.Add(ref csvLinear, ref csvAngular, out var csv);

            Matrix2x2Wide.TransformWithoutOverlap(ref csv, ref data.EffectiveMass, out var csi);

            var previousAccumulated = accumulatedImpulse;
            Vector2Wide.Add(ref accumulatedImpulse, ref csi, out accumulatedImpulse);
            //The maximum force of friction depends upon the normal impulse. The maximum is supplied per iteration.
            Vector2Wide.Length(ref accumulatedImpulse, out var accumulatedMagnitude);
            //Note division by zero guard.
            var scale = Vector.Min(Vector<float>.One, maximumImpulse / Vector.Max(new Vector<float>(1e-16f), accumulatedMagnitude));
            Vector2Wide.Scale(ref accumulatedImpulse, ref scale, out accumulatedImpulse);

            Vector2Wide.Subtract(ref accumulatedImpulse, ref previousAccumulated, out correctiveCSI);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref TangentFrictionProjection projection, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeJacobians(ref tangentX, ref tangentY, ref projection.OffsetA, ref projection.OffsetB, out var jacobians);
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection, ref jacobians, ref maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref jacobians, ref inertiaA, ref inertiaB, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
