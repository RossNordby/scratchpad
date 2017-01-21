using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Handles the solve iterations of a bunch of 1DOF two body inequality constraints.
    /// </summary>
    public class ContactPenetrationTypeBatch : TypeBatch<BodyReferences, ContactData, IterationData2Body1DOF, Vector<float>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobiansAndError(ref ContactData contact, out TwoBody1DOFJacobians jacobians, out Vector<float> error)
        {
            //Technically we could take advantage of the redundant form on the linear jacobians, but it's made complex by the pretransformations.
            //For now, just leave it fully generic and unspecialized.

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
            jacobians.LinearA = contact.Normal;
            Vector3Wide.Negate(ref contact.Normal, out jacobians.LinearB);
            Vector3Wide.CrossWithoutOverlap(ref contact.OffsetA, ref contact.Normal, out jacobians.AngularA);
            Vector3Wide.CrossWithoutOverlap(ref contact.Normal, ref contact.OffsetB, out jacobians.AngularB);

            //Note that we leave the penetration depth as is, even when it's negative. Speculative contacts!
            error = contact.PenetrationDepth;
        }

        public override void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt, int startBundle, int endBundle)
        {
            for (int i = startBundle; i < endBundle; ++i)
            {
                ComputeJacobiansAndError(ref PrestepData[i], out var jacobians, out var error);
                Inequality2Body1DOF.Prestep(bodyInertias, ref BodyReferences[i], ref IterationData[i], ref jacobians, ref PrestepData[i].SpringSettings, ref error, dt, inverseDt);
            }
        }
        public override void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            for (int i = startBundle; i < endBundle; ++i)
            {
                GatherScatter.GatherVelocities(bodyVelocities, ref BodyReferences[i], out var wsvA, out var wsvB);
                Inequality2Body1DOF.WarmStart(ref IterationData[i], ref AccumulatedImpulses[i], ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref BodyReferences[i], ref wsvA, ref wsvB);
            }
        }
        public override void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            for (int i = startBundle; i < endBundle; ++i)
            {
                GatherScatter.GatherVelocities(bodyVelocities, ref BodyReferences[i], out var wsvA, out var wsvB);
                Inequality2Body1DOF.Solve(ref IterationData[i], ref AccumulatedImpulses[i], ref wsvA, ref wsvB);
                GatherScatter.ScatterVelocities(bodyVelocities, ref BodyReferences[i], ref wsvA, ref wsvB);
            }
        }

        //TODO: This is strictly debug stuff.
        /// <summary>
        /// Performs an iteration in the batch, collecting velocity changes. This changes the state of the simulation!
        /// </summary>
        public void GetVelocityChanges(BodyVelocities[] velocities, ref BodyVelocities velocityChangesA, ref BodyVelocities velocityChangesB)
        {
            for (int i = 0; i < bundleCount; ++i)
            {
                GatherScatter.GatherVelocities(velocities, ref BodyReferences[i], out var wsvA, out var wsvB);
                Inequality2Body1DOF.ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref IterationData[i], ref AccumulatedImpulses[i], out var correctiveCSI);
                var previousA = wsvA;
                var previousB = wsvB;
                Inequality2Body1DOF.ApplyImpulse(ref IterationData[i], ref correctiveCSI, ref wsvA, ref wsvB);
                ref var bundleA = ref Unsafe.Add(ref velocityChangesA, i);
                ref var bundleB = ref Unsafe.Add(ref velocityChangesB, i);
                Vector3Wide.Subtract(ref wsvA.LinearVelocity, ref previousA.LinearVelocity, out bundleA.LinearVelocity);
                Vector3Wide.Subtract(ref wsvA.AngularVelocity, ref previousA.AngularVelocity, out bundleA.AngularVelocity);
                Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref previousB.LinearVelocity, out bundleB.LinearVelocity);
                Vector3Wide.Subtract(ref wsvB.AngularVelocity, ref previousB.AngularVelocity, out bundleB.AngularVelocity);
            }
        }
        public unsafe float GetVelocityChangeHeuristic(BodyVelocities[] velocities)
        {
            var velocityChangesVectorizedA = new BodyVelocities[bundleCount];
            var velocityChangesVectorizedB = new BodyVelocities[bundleCount];
            GetVelocityChanges(velocities, ref velocityChangesVectorizedA[0], ref velocityChangesVectorizedB[0]);

            //Clamp away any unfilled lanes.
            var lastBundleIndex = bundleCount - 1;
            var lastBundleCount = constraintCount - (lastBundleIndex << BundleIndexing.VectorShift);         
            var zeroVelocity = new BodyVelocity();
            for (int i = lastBundleCount; i < Vector<float>.Count; ++i)
            {
                GatherScatter.SetLane(ref velocityChangesVectorizedA[lastBundleIndex], i, ref zeroVelocity);
                GatherScatter.SetLane(ref velocityChangesVectorizedB[lastBundleIndex], i, ref zeroVelocity);
            }


            //This is not a particularly meaningful value in terms of physics. We just want to see that it reduces as more iterations are applied.
            float accumulatedChange = 0;
            for (int i = 0; i < bundleCount; ++i)
            {
                Vector3Wide.Dot(ref velocityChangesVectorizedA[i].LinearVelocity, ref velocityChangesVectorizedA[i].LinearVelocity, out var linearSquaredA);
                Vector3Wide.Dot(ref velocityChangesVectorizedA[i].AngularVelocity, ref velocityChangesVectorizedA[i].AngularVelocity, out var angularSquaredA);
                Vector3Wide.Dot(ref velocityChangesVectorizedB[i].LinearVelocity, ref velocityChangesVectorizedB[i].LinearVelocity, out var linearSquaredB);
                Vector3Wide.Dot(ref velocityChangesVectorizedB[i].AngularVelocity, ref velocityChangesVectorizedB[i].AngularVelocity, out var angularSquaredB);
                var velocityChange = Vector.SquareRoot(linearSquaredA) + Vector.SquareRoot(angularSquaredA) + Vector.SquareRoot(linearSquaredB) + Vector.SquareRoot(angularSquaredB);
                accumulatedChange += Vector.Dot(velocityChange, Vector<float>.One);
            }
            return accumulatedChange;
        }

    }
}
