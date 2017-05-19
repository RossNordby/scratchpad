//using BEPUutilities2.Memory;
//using System.Numerics;
//using System.Runtime.CompilerServices;

//namespace SolverPrototype.Constraints
//{
//    /// <summary>
//    /// Handles the solve iterations of a bunch of 1DOF two body inequality constraints.
//    /// </summary>
//    public class ContactPenetrationTypeBatch : TwoBodyTypeBatch<ContactData, Projection2Body1DOF, Vector<float>>
//    {

//        public override void Prestep(ref Buffer<BodyInertias> bodyInertias, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
//        {
//            for (int i = startBundle; i < exclusiveEndBundle; ++i)
//            {
//                ContactPenetrationLimit.ComputeJacobiansAndError(ref PrestepData[i], out var jacobians, out var error);
//                BodyReferences[i].Unpack(i, constraintCount, out var bodyReferences);
//                GatherScatter.GatherInertia(ref bodyInertias, ref bodyReferences, out var inertiaA, out var inertiaB);
//                Inequality2Body1DOF.Prestep(ref inertiaA, ref inertiaB, ref jacobians, ref PrestepData[i].SpringSettings, ref error, dt, inverseDt, out Projection[i]);
//            }
//        }
//        public override void WarmStart(ref Buffer<BodyVelocities> bodyVelocities, int startBundle, int exclusiveEndBundle)
//        {
//            for (int i = startBundle; i < exclusiveEndBundle; ++i)
//            {
//                BodyReferences[i].Unpack(i, constraintCount, out var bodyReferences);
//                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
//                Inequality2Body1DOF.WarmStart(ref Projection[i], ref AccumulatedImpulses[i], ref wsvA, ref wsvB);
//                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
//            }
//        }
//        public override void SolveIteration(ref Buffer<BodyVelocities> bodyVelocities, int startBundle, int exclusiveEndBundle)
//        {
//            for (int i = startBundle; i < exclusiveEndBundle; ++i)
//            {
//                BodyReferences[i].Unpack(i, constraintCount, out var bodyReferences);
//                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
//                Inequality2Body1DOF.Solve(ref Projection[i], ref AccumulatedImpulses[i], ref wsvA, ref wsvB);
//                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
//            }
//        }

//        //TODO: This is strictly debug stuff.
//        /// <summary>
//        /// Performs an iteration in the batch, collecting velocity changes. This changes the state of the simulation!
//        /// </summary>
//        public void GetVelocityChanges(ref Buffer<BodyVelocities> velocities, ref BodyVelocities velocityChangesA, ref BodyVelocities velocityChangesB)
//        {
//            for (int i = 0; i < bundleCount; ++i)
//            {
//                BodyReferences[i].Unpack(i, constraintCount, out var bodyReferences);
//                GatherScatter.GatherVelocities(ref velocities, ref bodyReferences, out var wsvA, out var wsvB);
//                Inequality2Body1DOF.ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref Projection[i], ref AccumulatedImpulses[i], out var correctiveCSI);
//                var previousA = wsvA;
//                var previousB = wsvB;
//                Inequality2Body1DOF.ApplyImpulse(ref Projection[i], ref correctiveCSI, ref wsvA, ref wsvB);
//                ref var bundleA = ref Unsafe.Add(ref velocityChangesA, i);
//                ref var bundleB = ref Unsafe.Add(ref velocityChangesB, i);
//                Vector3Wide.Subtract(ref wsvA.LinearVelocity, ref previousA.LinearVelocity, out bundleA.LinearVelocity);
//                Vector3Wide.Subtract(ref wsvA.AngularVelocity, ref previousA.AngularVelocity, out bundleA.AngularVelocity);
//                Vector3Wide.Subtract(ref wsvB.LinearVelocity, ref previousB.LinearVelocity, out bundleB.LinearVelocity);
//                Vector3Wide.Subtract(ref wsvB.AngularVelocity, ref previousB.AngularVelocity, out bundleB.AngularVelocity);
//            }
//        }
//        public unsafe float GetVelocityChangeHeuristic(ref Buffer<BodyVelocities> velocities)
//        {
//            var velocityChangesVectorizedA = new BodyVelocities[bundleCount];
//            var velocityChangesVectorizedB = new BodyVelocities[bundleCount];
//            GetVelocityChanges(ref velocities, ref velocityChangesVectorizedA[0], ref velocityChangesVectorizedB[0]);

//            //Clamp away any unfilled lanes.
//            var lastBundleIndex = bundleCount - 1;
//            var lastBundleCount = constraintCount - (lastBundleIndex << BundleIndexing.VectorShift);         
//            var zeroVelocity = new BodyVelocity();
//            for (int i = lastBundleCount; i < Vector<float>.Count; ++i)
//            {
//                Bodies.SetLane(ref velocityChangesVectorizedA[lastBundleIndex], i, ref zeroVelocity);
//                Bodies.SetLane(ref velocityChangesVectorizedB[lastBundleIndex], i, ref zeroVelocity);
//            }


//            //This is not a particularly meaningful value in terms of physics. We just want to see that it reduces as more iterations are applied.
//            float accumulatedChange = 0;
//            for (int i = 0; i < bundleCount; ++i)
//            {
//                Vector3Wide.Dot(ref velocityChangesVectorizedA[i].LinearVelocity, ref velocityChangesVectorizedA[i].LinearVelocity, out var linearSquaredA);
//                Vector3Wide.Dot(ref velocityChangesVectorizedA[i].AngularVelocity, ref velocityChangesVectorizedA[i].AngularVelocity, out var angularSquaredA);
//                Vector3Wide.Dot(ref velocityChangesVectorizedB[i].LinearVelocity, ref velocityChangesVectorizedB[i].LinearVelocity, out var linearSquaredB);
//                Vector3Wide.Dot(ref velocityChangesVectorizedB[i].AngularVelocity, ref velocityChangesVectorizedB[i].AngularVelocity, out var angularSquaredB);
//                var velocityChange = Vector.SquareRoot(linearSquaredA) + Vector.SquareRoot(angularSquaredA) + Vector.SquareRoot(linearSquaredB) + Vector.SquareRoot(angularSquaredB);
//                accumulatedChange += Vector.Dot(velocityChange, Vector<float>.One);
//            }
//            return accumulatedChange;
//        }

//    }
//}
