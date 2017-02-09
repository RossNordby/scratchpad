using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation of memory moves for all two body constraints.
    /// </summary>
    public abstract class TwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse> : TypeBatch<TwoBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>
    {
        public sealed override void GetConnectedBodyIndices<TEnumerator>(int indexInTypeBatch, ref TEnumerator enumerator)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);

            ref var bundleIndexA = ref GatherScatter.Get(ref BodyReferences[constraintBundleIndex].BundleIndexA, constraintInnerIndex);
            ref var innerIndexA = ref Unsafe.Add(ref bundleIndexA, Vector<int>.Count);
            ref var bundleIndexB = ref Unsafe.Add(ref bundleIndexA, 2 * Vector<int>.Count);
            ref var innerIndexB = ref Unsafe.Add(ref bundleIndexA, 3 * Vector<int>.Count);

            //Note that the variables are ref locals! This is important for correctness, because every execution of LoopBody could result in a swap.
            //Ref locals aren't the only solution, but if you ever change this, make sure you account for the potential mutation in the enumerator.
            enumerator.LoopBody((bundleIndexA << BundleIndexing.VectorShift) | innerIndexA);
            enumerator.LoopBody((bundleIndexB << BundleIndexing.VectorShift) | innerIndexB);
        }
        public sealed override void UpdateForBodyMemoryMove(int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation)
        {
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            BundleIndexing.GetBundleIndices(newBodyLocation, out var bodyBundleIndex, out var bodyInnerIndex);

            //Note that this relies on the bodyreferences memory layout. It uses the stride of vectors to skip to the next body based on the bodyIndexInConstraint.
            ref var bundleIndex = ref GatherScatter.Get(ref BodyReferences[constraintBundleIndex].BundleIndexA, bodyInnerIndex + bodyIndexInConstraint * (Vector<int>.Count * 2));
            ref var innerIndex = ref Unsafe.Add(ref bundleIndex, Vector<int>.Count);
            bundleIndex = bodyBundleIndex;
            innerIndex = bodyInnerIndex;
        }
    }
}
