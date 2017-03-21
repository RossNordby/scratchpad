using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{
    //Not a big fan of complex generic-filled inheritance hierarchies, but this is the shortest evolutionary step to removing duplicates.
    //There are some other options if this inheritance hierarchy gets out of control.
    /// <summary>
    /// Shared implementation of memory moves for all two body constraints.
    /// </summary>
    public abstract class TwoBodyTypeBatch<TPrestepData, TProjection, TAccumulatedImpulse> : TypeBatch<TwoBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>
    {
        public sealed override int BodiesPerConstraint => 2;

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

        protected unsafe sealed override void AddBodyReferences(int index, int* bodyIndices)
        {
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            ref var bundle = ref BodyReferences[bundleIndex];
            Debug.Assert(innerIndex == 0 || bundle.Count == innerIndex,
                "Either this bundle hasn't been initialized yet (and so has unknown count), or it should match the new inner index.");
            //Since we only ever add constraints at the very end, the count is based on the inner index.
            bundle.Count = innerIndex + 1;
            GatherScatter.SetBodyReferencesLane(ref bundle.BundleIndexA, innerIndex, bodyIndices, 2);
#if DEBUG
            for (int i = 0; i < bundle.Count - 1; ++i)
            {
                var aIndex = (bundle.BundleIndexA[i] << BundleIndexing.VectorShift) | bundle.InnerIndexA[i];
                var bIndex = (bundle.BundleIndexB[i] << BundleIndexing.VectorShift) | bundle.InnerIndexB[i];
                for (int j = i + 1; j < bundle.Count; ++j)
                {
                    var aIndex2 = (bundle.BundleIndexA[j] << BundleIndexing.VectorShift) | bundle.InnerIndexA[j];
                    var bIndex2 = (bundle.BundleIndexB[j] << BundleIndexing.VectorShift) | bundle.InnerIndexB[j];
                    Debug.Assert(!(
                        aIndex == bIndex || aIndex2 == bIndex2 ||
                        aIndex == aIndex2 || aIndex == bIndex2 ||
                        bIndex == aIndex2 || bIndex == bIndex2),
                        "A bundle should not share any body references. If an add causes redundant body references, something upstream broke.");
                }

            }
#endif

        }
        protected sealed override void RemoveBodyReferences(int bundleIndex, int innerIndex)
        {
            ref var bundle = ref BodyReferences[bundleIndex];
            Debug.Assert(bundle.Count > 0, "The caller should guarantee that any one bundle isn't over-removed from.");
            //This is a little defensive; in the event that the body set actually scales down, you don't want to end up with invalid pointers in some lanes.
            //TODO: This may need to change depending on how we handle kinematic/static/inactive storage and encoding.
            //Note the use of an explicit count. We don't directly control the memory size of TwoBodyReferences, but we don't want the clear to touch the count slot. 
            GatherScatter.ClearLane<TwoBodyReferences, int>(ref bundle, innerIndex, 2);
            bundle.Count--;

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
        struct IntComparer : IComparerRef<int>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref int a, ref int b)
            {
                return a > b ? 1 : a < b ? -1 : 0;
            }
        }
        public override sealed void SortByBodyLocation(int bundleStartIndex, int constraintCount, ConstraintLocation[] handlesToConstraints, int bodyCount)
        {
            int bundleCount = (constraintCount >> BundleIndexing.VectorShift);
            if ((constraintCount & BundleIndexing.VectorMask) != 0)
                ++bundleCount;

            //TODO: Replace these buffer pools with new buffer pools once they're ready. Probably passed in from above to guarantee thread safety.
            //(New pools are typeless, so no issue there. The pointer backing means we'll probably have to shift the below to blockcopies, but that's fine.)
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
            Array.Copy(IndexToHandle, bundleStartIndex * Vector<int>.Count, handlesCache, 0, constraintCount);

            //First, compute the proper order of the constraints in this region by sorting their keys.
            //This minimizes the number of swaps that must be applied to the actual bundle data.
            //Avoiding swaps of actual data is very valuable- a single constraint can be hundreds of bytes, and the accesses to a slot tend to require some complex addressing.
            var baseIndex = bundleStartIndex * Vector<int>.Count;
            for (int i = 0; i < constraintCount; ++i)
            {
                sourceIndices[i] = i;
                sortKeys[i] = GetSortKey(baseIndex + i);
            }

            //TODO: Later on, if you switch to pointer or ambiguously backed memory, you'll have to use a different sort.
            //If some form of span.sort doesn't exist yet, we'll need to use our own little sort implementation. The following works reasonably well,
            //though it is a little slower than the default sort in some scrambled cases. This isn't a big concern.
            //TODO: You may want to actually change the way the optimizer handles sorting. Rather than doing a bunch of independent sorts, it could do a cooperative sort.
            //It would be slower in terms of raw coverage, but it would converge faster. The key there would be an efficient and parallel merge operation.
            //Subarrays could be sorted by any available means.
            //var comparer = default(IntComparer);
            //Quicksort.Sort2(ref sortKeys[0], ref sourceIndices[0], 0, constraintCount - 1, ref comparer);

            Array.Sort(sortKeys, sourceIndices, 0, constraintCount);

            //Push the cached data into its proper sorted position.
            for (int i = 0; i < constraintCount; ++i)
            {
                var sourceIndex = sourceIndices[i];
                var targetIndex = baseIndex + i;
                //Note that we do not bother checking whether the source and target are the same.
                //The cost of the branch is large enough in comparison to the frequency of its usefulness that it only helps in practically static situations.
                //Also, its maximum benefit is quite small.
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

                Move(ref referencesCache[sourceBundle], ref prestepCache[sourceBundle], ref accumulatedImpulseCache[sourceBundle],
                    sourceInner, handlesCache[sourceIndex],
                    targetBundle, targetInner, targetIndex, handlesToConstraints);

            }



            BufferPools<int>.Locking.Return(sourceIndices);
            BufferPools<int>.Locking.Return(sortKeys);
            BufferPools<int>.Locking.Return(handlesCache);
            BufferPools<TwoBodyReferences>.Locking.Return(referencesCache);
            BufferPools<TPrestepData>.Locking.Return(prestepCache);
            BufferPools<TAccumulatedImpulse>.Locking.Return(accumulatedImpulseCache);
        }

        public override void ValidateBundleCounts()
        {
            //This is a pure debug function. Performance does not matter.
            for (int i = 0; i < bundleCount - 1; ++i)
            {
                Debug.Assert(BodyReferences[i].Count == Vector<float>.Count, "All bundles prior to the last bundle should be full.");
            }
            if (bundleCount > 0)
                Debug.Assert(BodyReferences[bundleCount - 1].Count == constraintCount - (bundleCount - 1) * Vector<float>.Count, "The last bundle should contain the remainder.");
            else
                Debug.Assert(constraintCount == 0);
            Debug.Assert(bundleCount == BodyReferences.Length || BodyReferences[bundleCount].Count == 0, "Body references beyond the end should have 0 count.");
        }
    }
}
