//using BEPUutilities2.Memory;
//using System;
//using System.Diagnostics;
//using System.Numerics;
//using System.Runtime.CompilerServices;

//namespace SolverPrototype.Constraints
//{
//    public struct BallSocketPrestepData
//    {
//        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
//        //If you modify this layout, be sure to update the associated BallSocketConstraint.
//        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
//        public SpringSettings SpringSettings;
//    }

//    public struct BallSocketProjection
//    {
//    }

//    /// <summary>
//    /// Handles the solve iterations of a bunch of ball socket constraints.
//    /// </summary>
//    public class BallSocketTypeBatch : TwoBodyTypeBatch<BallSocketPrestepData, BallSocketProjection, Vector3Wide>
//    {
//        struct BallSocketPrestep : IPosedPrestep
//        {
//            public void Prestep(ref BallSocketPrestepData prestepData, 
//                ref BodyInertias inertiaA, ref BodyInertias inertiaB, 
//                ref Vector3Wide relativePosition, ref QuaternionWide orientationA, ref QuaternionWide orientationB, 
//                out BallSocketProjection projection)
//            {
//                projection = new BallSocketProjection();
//            }
//        }

//        public override void Prestep(Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
//        {
//            PosedPrestep<BallSocketPrestep>(bodies, dt, inverseDt, startBundle, exclusiveEndBundle);
//        }
//        public override void WarmStart(ref Buffer<BodyVelocities> bodyVelocities, int startBundle, int exclusiveEndBundle)
//        {
//            ref var bodyReferencesBase = ref BodyReferences[0];
//            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
//            ref var projectionBase = ref Projection[0];
//            for (int i = startBundle; i < exclusiveEndBundle; ++i)
//            {
//                ref var projection = ref Unsafe.Add(ref projectionBase, i);
//                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
//                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);
//                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
//                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
//            }
//        }

//        public override void SolveIteration(ref Buffer<BodyVelocities> bodyVelocities, int startBundle, int exclusiveEndBundle)
//        {
//            ref var projectionBase = ref Projection[0];
//            ref var bodyReferencesBase = ref BodyReferences[0];
//            ref var accumulatedImpulsesBase = ref AccumulatedImpulses[0];
//            for (int i = startBundle; i < exclusiveEndBundle; ++i)
//            {
//                ref var projection = ref Unsafe.Add(ref projectionBase, i);
//                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
//                ref var accumulatedImpulses = ref Unsafe.Add(ref accumulatedImpulsesBase, i);

//                GatherScatter.GatherVelocities(ref bodyVelocities, ref bodyReferences, out var wsvA, out var wsvB);
//                GatherScatter.ScatterVelocities(ref bodyVelocities, ref bodyReferences, ref wsvA, ref wsvB);
//            }
//        }

//    }
//}
