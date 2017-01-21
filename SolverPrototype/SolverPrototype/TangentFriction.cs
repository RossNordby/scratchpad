using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    //For in depth explanations of constraints, check the Inequality1DOF.cs implementation.
    //The details are omitted for brevity in other implementations.
    public struct Jacobians2Body2DOF
    {
        public Matrix2x3Wide LinearA;
        public Matrix2x3Wide AngularA;
        public Matrix2x3Wide LinearB;
        public Matrix2x3Wide AngularB;
    }


    public struct TangentFrictionIterationData
    {
        public Matrix2x3Wide WSVtoCSILinearA;
        public Matrix2x3Wide WSVtoCSIAngularA;
        public Matrix2x3Wide WSVtoCSILinearB;
        public Matrix2x3Wide WSVtoCSIAngularB;

        public Matrix2x3Wide CSIToWSVLinearA;
        public Matrix2x3Wide CSIToWSVAngularA;
        public Matrix2x3Wide CSIToWSVLinearB;
        public Matrix2x3Wide CSIToWSVAngularB;
    }

    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TangentFriction
    {
        //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(ref Vector3Wide tangentX, ref Vector3Wide tangentY, ref Vector3Wide offsetA, ref Vector3Wide offsetB, out Jacobians2Body2DOF jacobians)
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
            Vector3Wide.CrossWithoutOverlap(ref offsetA, ref tangentY, out jacobians.AngularA.Y);
            Matrix2x3Wide.Negate(ref jacobians.LinearA, out jacobians.LinearB);
            Vector3Wide.CrossWithoutOverlap(ref tangentX, ref offsetB, out jacobians.AngularB.X);
            Vector3Wide.CrossWithoutOverlap(ref tangentY, ref offsetB, out jacobians.AngularB.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Jacobians2Body2DOF jacobians,
            out TangentFrictionIterationData data)
        {
            //Compute effective mass matrix contributions.
            Matrix2x3Wide.Scale(ref jacobians.LinearA, ref inertiaA.InverseMass, out data.CSIToWSVLinearA);
            Matrix2x3Wide.Scale(ref jacobians.LinearB, ref inertiaB.InverseMass, out data.CSIToWSVLinearB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref data.CSIToWSVLinearA, ref jacobians.LinearA, out var linearA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref data.CSIToWSVLinearB, ref jacobians.LinearB, out var linearB);

            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularA, ref inertiaA.InverseInertiaTensor, out data.CSIToWSVAngularA);
            Matrix2x3Wide.MultiplyWithoutOverlap(ref jacobians.AngularB, ref inertiaB.InverseInertiaTensor, out data.CSIToWSVAngularB);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref data.CSIToWSVAngularA, ref jacobians.AngularA, out var angularA);
            Matrix2x3Wide.MultiplyByTransposeWithoutOverlap(ref data.CSIToWSVAngularB, ref jacobians.AngularB, out var angularB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Matrix2x2Wide.Add(ref linearA, ref linearB, out var linear);
            Matrix2x2Wide.Add(ref angularA, ref angularB, out var angular);
            Matrix2x2Wide.Add(ref linear, ref angular, out var inverseEffectiveMass);
            Matrix2x2Wide.Invert(ref inverseEffectiveMass, out var effectiveMass);

            //Note that friction constraints have no bias velocity. They target zero velocity.

            //Finally, compute the transform for constraint space impulse to world space.
            //This is precomputing wsv * (JT * effectiveMass).
            //Note that we only have a 2x3 representation, so we are actually storing (JT * effectiveMass)T = effectiveMassT * J.
            //During the solve, we use wsv * (effectiveMassT * J)T.
            Matrix2x3Wide.MultiplyTransposedWithoutOverlap(ref effectiveMass, ref jacobians.LinearA, out data.WSVtoCSILinearA);
            Matrix2x3Wide.MultiplyTransposedWithoutOverlap(ref effectiveMass, ref jacobians.AngularA, out data.WSVtoCSIAngularA);
            Matrix2x3Wide.MultiplyTransposedWithoutOverlap(ref effectiveMass, ref jacobians.LinearB, out data.WSVtoCSILinearB);
            Matrix2x3Wide.MultiplyTransposedWithoutOverlap(ref effectiveMass, ref jacobians.AngularB, out data.WSVtoCSIAngularB);
        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        public static void ApplyImpulse(ref TangentFrictionIterationData data, ref Vector2Wide correctiveImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            BodyVelocities correctiveVelocityA, correctiveVelocityB;
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref data.CSIToWSVLinearA, out correctiveVelocityA.LinearVelocity);
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref data.CSIToWSVAngularA, out correctiveVelocityA.AngularVelocity);
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref data.CSIToWSVLinearB, out correctiveVelocityB.LinearVelocity);
            Matrix2x3Wide.Transform(ref correctiveImpulse, ref data.CSIToWSVAngularB, out correctiveVelocityB.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.LinearVelocity, ref wsvA.LinearVelocity, out wsvA.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityA.AngularVelocity, ref wsvA.AngularVelocity, out wsvA.AngularVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.LinearVelocity, ref wsvB.LinearVelocity, out wsvB.LinearVelocity);
            Vector3Wide.Add(ref correctiveVelocityB.AngularVelocity, ref wsvB.AngularVelocity, out wsvB.AngularVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref TangentFrictionIterationData data, ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(ref data, ref accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref TangentFrictionIterationData data, ref Vector<float> maximumImpulse,
            ref Vector2Wide accumulatedImpulse, out Vector2Wide correctiveCSI)
        {
            //Recall that our WSVtoCSI... is transposed due to the lack of a 3x2 representation.
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.LinearVelocity, ref data.WSVtoCSILinearA, out var csiaLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvA.AngularVelocity, ref data.WSVtoCSIAngularA, out var csiaAngular);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvB.LinearVelocity, ref data.WSVtoCSILinearB, out var csibLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref wsvB.AngularVelocity, ref data.WSVtoCSIAngularB, out var csibAngular);
            Vector2Wide.Add(ref csiaLinear, ref csiaAngular, out var csia);
            Vector2Wide.Add(ref csibLinear, ref csibAngular, out var csib);
            Vector2Wide.Add(ref csia, ref csib, out var negativeCSI); //Since there is no bias or softness to give us the negative, we just do it when we apply to the accumulated impulse.

            var previousAccumulated = accumulatedImpulse;
            Vector2Wide.Subtract(ref accumulatedImpulse, ref negativeCSI, out accumulatedImpulse);
            //The maximum force of friction depends upon the normal impulse. The maximum is supplied per iteration.
            Vector2Wide.Length(ref accumulatedImpulse, out var accumulatedMagnitude);
            var scale = Vector.Min(Vector<float>.One, maximumImpulse / accumulatedMagnitude);
            Vector2Wide.Scale(ref accumulatedImpulse, ref scale, out accumulatedImpulse);

            Vector2Wide.Subtract(ref accumulatedImpulse, ref previousAccumulated, out correctiveCSI);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref TangentFrictionIterationData data, ref Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref data, ref maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref data, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
