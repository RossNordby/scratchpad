using System;
using System.Diagnostics;
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
        public sealed override void EnumerateConnectedBodyIndices<TEnumerator>(int indexInTypeBatch, ref TEnumerator enumerator)
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
            ref var bundleIndex = ref GatherScatter.Get(ref BodyReferences[constraintBundleIndex].BundleIndexA, constraintInnerIndex + bodyIndexInConstraint * (Vector<int>.Count * 2));
            ref var innerIndex = ref Unsafe.Add(ref bundleIndex, Vector<int>.Count);
            bundleIndex = bodyBundleIndex;
            innerIndex = bodyInnerIndex;
        }

        public sealed override void SortByBodyLocation(int startIndex, int count, ConstraintLocation[] handlesToConstraints)
        {
            int GetSortKey(int constraintIndex)
            {
                BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
                ref var bundleReferences = ref BodyReferences[bundleIndex];
                Debug.Assert(bundleIndex < bundleCount && innerIndex < bundleReferences.Count);
                //We sort based on the body references within the constraint. 
                //Sort based on the smaller body index in a constraint. Note that it is impossible for there to be two references to the same body within a constraint batch, 
                //so there's no need to worry about the case where the comparison is equal.
                //TODO: Even without any other higher level optimizations, this does some unnecessary recasting. We can walk by stride.
                var bodyIndexA = (GatherScatter.Get(ref bundleReferences.BundleIndexA, innerIndex) << BundleIndexing.VectorShift) | GatherScatter.Get(ref bundleReferences.InnerIndexA, innerIndex);
                var bodyIndexB = (GatherScatter.Get(ref bundleReferences.BundleIndexB, innerIndex) << BundleIndexing.VectorShift) | GatherScatter.Get(ref bundleReferences.InnerIndexB, innerIndex);
                return bodyIndexA < bodyIndexB ? bodyIndexA : bodyIndexB;
            }
            //We'll cache the element being swapped down on the stack in the first lane of these instances.
            //You could be a little more efficient here- this will zero out Vector<float>.Count times more memory than we actually use.
            //You could stackalloc sizeof(TPrestepData etc) bytes and treat it as a copyable lane instead. That's getting into some pretty questionable territory!
            TwoBodyReferences referencesCache = default(TwoBodyReferences);
            TPrestepData prestepCache = default(TPrestepData);
            TAccumulatedImpulse accumulatedImpulseCache = default(TAccumulatedImpulse);
            //Under the assumption that the sort region is going to be pretty small and often nearly sorted, use insertion sort.
            var endIndex = startIndex + count;
            Debug.Assert(endIndex <= constraintCount, "Bad startIndex or count; the caller is responsible for validating the input.");
            for (int constraintIndex = startIndex + 1; constraintIndex < endIndex; ++constraintIndex)
            {
                //TODO: It may be possible to significantly improve the worst case by scanning with SIMD. Compare whole bundles simultaneously.
                //This would be primarily useful for large movements where it may need to swap across many bundles. Not sure if our sort region is large enough to ever get a win.
                //Only do that after you have a fully scalar version working, and only if the performance of optimization is at least remotely concerning.
                //Note that the swaps will likely dominate the initial search completely.
                var sortKey = GetSortKey(constraintIndex);

                //TODO: Note redundant calculation of bundle indices.
                BundleIndexing.GetBundleIndices(constraintIndex, out var constraintBundle, out var constraintInner);
                //TODO: Could do conditional caching if you're willing to grab the index and do a separate loop afterwards.
                GatherScatter.CopyLane(ref BodyReferences[constraintBundle], constraintInner, ref referencesCache, 0);
                GatherScatter.CopyLane(ref AccumulatedImpulses[constraintBundle], constraintInner, ref accumulatedImpulseCache, 0);
                GatherScatter.CopyLane(ref PrestepData[constraintBundle], constraintInner, ref prestepCache, 0);
                var handleCache = Handles[constraintIndex];
                int targetIndex = constraintIndex;
                for (int compareIndex = constraintIndex - 1; compareIndex >= startIndex; --compareIndex)
                {
                    var comparisonKey = GetSortKey(compareIndex);
                    if (comparisonKey < sortKey)
                    {
                        //The target index we set last is as far as we had to go.
                        break;
                    }
                    else
                    {
                        //This constraint is still larger than the constraint being checked for swap potential.

                        //TODO: Note redundant calculation of bundle indices. Could save one execution by sharing with the getsortkey, and the other by using a 'previous' cache.
                        Move(compareIndex, compareIndex + 1, handlesToConstraints);
                        targetIndex = compareIndex;
                    }
                }
                if (targetIndex < constraintIndex)
                {
                    //The cached element was overwritten and a new slot was found for it. Copy the cached data into it.
                    BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);
                    Move(ref referencesCache, ref prestepCache, ref accumulatedImpulseCache, 0, handleCache, targetBundle, targetInner, targetIndex, handlesToConstraints);
                }
            }
        }
    }
}
