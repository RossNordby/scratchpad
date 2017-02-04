using BEPUutilities2.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation of memory moves for all two body constraints.
    /// </summary>
    public abstract class TwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse> : TypeBatch<TwoBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>
    {

        public sealed override void GetConnectedBodyIndices(int indexInTypeBatch, ref QuickList<int> bodyIndices)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);

            ref var bundleIndexA = ref GatherScatter.Get(ref BodyReferences[constraintBundleIndex].BundleIndexA, constraintInnerIndex);
            ref var innerIndexA = ref Unsafe.Add(ref bundleIndexA, Vector<int>.Count);
            ref var bundleIndexB = ref Unsafe.Add(ref bundleIndexA, 2 * Vector<int>.Count);
            ref var innerIndexB = ref Unsafe.Add(ref bundleIndexA, 3 * Vector<int>.Count);

            bodyIndices.Add((bundleIndexA << BundleIndexing.VectorShift) | innerIndexA);
            bodyIndices.Add((bundleIndexB << BundleIndexing.VectorShift) | innerIndexB);
        }
        public sealed override void UpdateForBodyMemoryMove(int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var bodyBundleIndex, out var bodyInnerIndex);

            //Note that this relies on the bodyreferences memory layout. It uses the stride of vectors to skip to the next body based on the bodyIndexInConstraint.
            ref var bundleIndex = ref GatherScatter.Get(ref BodyReferences[constraintBundleIndex].BundleIndexA, bodyInnerIndex + bodyIndexInConstraint * (Vector<int>.Count * 2));
            ref var innerIndex = ref Unsafe.Add(ref bundleIndex, Vector<int>.Count);
            bundleIndex = bodyBundleIndex;
            innerIndex = bodyInnerIndex;
        }
    }
}
