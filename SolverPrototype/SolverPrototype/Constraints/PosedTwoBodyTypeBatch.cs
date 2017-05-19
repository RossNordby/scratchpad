//using System;
//using System.Collections.Generic;
//using System.Runtime.CompilerServices;
//using System.Text;

//namespace SolverPrototype.Constraints
//{
//    public abstract class PosedTwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse, 
//        TPrestepFunction, TWarmStartFunction, TSolveFunction> : TwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse, TWarmStartFunction, TSolveFunction>
//        where TPrestepFunction : IPosedPrestep

//    {
//        //The following handle the looping and gather logic common to all two body constraints with zero overhead, so long as the interface implementations
//        //are all aggressively inlined. Saves quite a bit of performance sensitive and error prone duplicate code.

//        /// <summary>
//        /// Defines a bundle prestep function for use with constraints which require the current body pose as input.
//        /// </summary>
//        protected interface IPosedPrestep
//        {
//            void Prestep(ref TPrestepData prestepData, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
//                ref Vector3Wide relativePosition, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
//                out TProjection projection);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        protected void PosedPrestep<T>(Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle) where T : struct, IPosedPrestep
//        {
//            ref var prestepBase = ref PrestepData[0];
//            ref var bodyReferencesBase = ref BodyReferences[0];
//            ref var projectionBase = ref Projection[0];
//            var function = default(T);
//            for (int i = startBundle; i < exclusiveEndBundle; ++i)
//            {
//                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
//                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
//                ref var projection = ref Unsafe.Add(ref projectionBase, i);
//                GatherScatter.GatherInertiaAndConstraintPose(ref bodies.Inertias, ref bodies.Poses, ref bodyReferences,
//                    out var relativePosition, out var orientationA, out var orientationB,
//                    out var inertiaA, out var inertiaB);
//                function.Prestep(ref prestep,
//                    ref inertiaA, ref inertiaB,
//                    ref relativePosition, ref orientationA, ref orientationB,
//                    out projection);

//            }



//    }
//}
