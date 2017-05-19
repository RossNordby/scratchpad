using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;
using BEPUutilities2.Memory;

namespace SolverPrototype.Constraints
{
    /// <summary>
    /// Defines a bundle prestep function for use with constraints which do not require the current body pose as input.
    /// </summary>
    public interface IUnposedPrestep<TPrestepData, TProjection>
    {
        void Prestep(ref TPrestepData prestepData, ref BodyInertias inertiaA, ref BodyInertias inertiaB, float dt, float inverseDt, out TProjection projection);
    }
    public abstract class UnposedTwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse,
        TPrestepFunction, TWarmStartAndSolveFunction> : TwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse, TWarmStartAndSolveFunction>
        where TPrestepFunction : struct, IUnposedPrestep<TPrestepData, TProjection>
        where TWarmStartAndSolveFunction : struct, IWarmStartAndSolve<TProjection, TAccumulatedImpulse>
    {
        //The following handle the looping and gather logic common to all two body constraints with zero overhead, so long as the interface implementations
        //are all aggressively inlined. Saves quite a bit of performance sensitive and error prone duplicate code.

        public override void Prestep(Bodies bodies, float dt, float inverseDt, int startBundle, int exclusiveEndBundle)
        {
            ref var prestepBase = ref PrestepData[0];
            ref var bodyReferencesBase = ref BodyReferences[0];
            ref var projectionBase = ref Projection[0];
            var function = default(TPrestepFunction);
            for (int i = startBundle; i < exclusiveEndBundle; ++i)
            {
                ref var prestep = ref Unsafe.Add(ref prestepBase, i);
                Unsafe.Add(ref bodyReferencesBase, i).Unpack(i, constraintCount, out var bodyReferences);
                ref var projection = ref Unsafe.Add(ref projectionBase, i);
                GatherScatter.GatherInertia(ref bodies.Inertias, ref bodyReferences, out var inertiaA, out var inertiaB);
                function.Prestep(ref prestep,
                    ref inertiaA, ref inertiaB,
                    dt, inverseDt,
                    out projection);

            }
        }
    }
}
