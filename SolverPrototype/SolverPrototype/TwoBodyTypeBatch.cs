using BEPUutilities2.ResourceManagement;
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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int GetSortKey(int constraintIndex)
        {
            BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
            ref var bundleReferences = ref BodyReferences[bundleIndex];
            Debug.Assert(bundleIndex < base.bundleCount && innerIndex < bundleReferences.Count);
            //We sort based on the body references within the constraint. 
            //Sort based on the smaller body index in a constraint. Note that it is impossible for there to be two references to the same body within a constraint batch, 
            //so there's no need to worry about the case where the comparison is equal.
            ref var bundleIndexA = ref GatherScatter.Get(ref bundleReferences.BundleIndexA, innerIndex);
            ref var innerIndexA = ref Unsafe.Add(ref bundleIndexA, Vector<int>.Count);
            ref var bundleIndexB = ref Unsafe.Add(ref bundleIndexA, 2 * Vector<int>.Count);
            ref var innerIndexB = ref Unsafe.Add(ref bundleIndexA, 3 * Vector<int>.Count);
            var bodyIndexA = (bundleIndexA << BundleIndexing.VectorShift) | innerIndexA;
            var bodyIndexB = (bundleIndexB << BundleIndexing.VectorShift) | innerIndexB;
            return bodyIndexA < bodyIndexB ? bodyIndexA : bodyIndexB;
        }
        public override sealed void SortByBodyLocation(int bundleStartIndex, int constraintCount, ConstraintLocation[] handlesToConstraints)
        {

            //These buffers could be handled more cleanly. You only really need to grab them once externally, and there's no need to use the old type-redundant system.
            //That would also allow pointers.
            int bundleCount = (constraintCount >> BundleIndexing.VectorShift);
            if ((constraintCount & BundleIndexing.VectorMask) != 0)
                ++bundleCount;
            var sourceIndices = BufferPools<int>.Locking.Take(constraintCount);
            var sortKeys = BufferPools<int>.Locking.Take(constraintCount);
            var handlesCache = BufferPools<int>.Locking.Take(constraintCount);
            var referencesCache = BufferPools<TwoBodyReferences>.Locking.Take(bundleCount);
            var prestepCache = BufferPools<TPrestepData>.Locking.Take(bundleCount);
            var accumulatedImpulseCache = BufferPools<TAccumulatedImpulse>.Locking.Take(bundleCount);

            //Cache the unsorted data.
            Array.Copy(BodyReferences, bundleStartIndex, referencesCache, 0, bundleCount);
            Array.Copy(PrestepData, bundleStartIndex, prestepCache, 0, bundleCount);
            Array.Copy(AccumulatedImpulses, bundleStartIndex, accumulatedImpulseCache, 0, bundleCount);
            Array.Copy(Handles, bundleStartIndex * Vector<int>.Count, handlesCache, 0, constraintCount);

            //First, compute the proper order of the constraints in this region by sorting their keys.
            //This minimizes the number of swaps that must be applied to the actual bundle data.
            //Avoiding swaps of actual data is very valuable- a single constraint can be hundreds of bytes, and the accesses to a slot tend to require some complex addressing.
            var baseIndex = bundleStartIndex * Vector<int>.Count;
            for (int i = 0; i < constraintCount; ++i)
            {
                sourceIndices[i] = i;
                sortKeys[i] = GetSortKey(baseIndex + i);
            }

            //TODO: Try insertion sort. The sort regions are often small, and after an initial chaotic period, they are often mostly sorted.
            Array.Sort(sortKeys, sourceIndices, 0, constraintCount);

            //TODO: There's a possibility that doing a coherent read -> incoherent scatter would be better than doing an incoherent gather -> coherent scatter.

            //Push the cached data into its proper sorted position.
            for (int i = 0; i < constraintCount; ++i)
            {
                BundleIndexing.GetBundleIndices(sourceIndices[i], out var sourceBundle, out var sourceInner);
                var targetIndex = baseIndex + i;
                BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

                Move(ref referencesCache[sourceBundle], ref prestepCache[sourceBundle], ref accumulatedImpulseCache[sourceBundle],
                    sourceInner, handlesCache[sourceIndices[i]],
                    targetBundle, targetInner, targetIndex, handlesToConstraints);
            }



            BufferPools<int>.Locking.Return(sourceIndices);
            BufferPools<int>.Locking.Return(sortKeys);
            BufferPools<int>.Locking.Return(handlesCache);
            BufferPools<TwoBodyReferences>.Locking.Return(referencesCache);
            BufferPools<TPrestepData>.Locking.Return(prestepCache);
            BufferPools<TAccumulatedImpulse>.Locking.Return(accumulatedImpulseCache);
        }
    }
}
