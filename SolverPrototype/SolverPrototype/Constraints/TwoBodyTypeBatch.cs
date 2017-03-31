﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
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

        protected sealed override void RemoveBodyReferences(int bundleIndex, int innerIndex)
        {
            ref var bundle = ref BodyReferences[bundleIndex];
            //This removal should only ever occur at the very end of the constraint set, so the innerIndex should be equal to the count + 1.
            Debug.Assert(bundle.Count == innerIndex + 1, "Since the last element is the only one that is ever removed, the last slot of the bundle should match the provided inner index.");
            //This is a little defensive; in the event that the body set actually scales down, you don't want to end up with invalid pointers in some lanes.
            //TODO: This may need to change depending on how we handle kinematic/static/inactive storage and encoding.
            //Note the use of an explicit count. We don't directly control the memory size of TwoBodyReferences, but we don't want the clear to touch the count slot. 
            //Note that the count is 4, not 2! The count is in terms of cleared slots, not bodies, and the body references struct has 2 slots per body (bundle and inner).
            GatherScatter.ClearLane<TwoBodyReferences, int>(ref bundle, innerIndex, 4);
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
        public override sealed void SortByBodyLocation(int bundleStartIndex, int constraintCount, ConstraintLocation[] handlesToConstraints, int bodyCount, BufferPool rawPool)
        {
            int bundleCount = (constraintCount >> BundleIndexing.VectorShift);
            if ((constraintCount & BundleIndexing.VectorMask) != 0)
                ++bundleCount;
            
            rawPool.SpecializeFor<int>().Take(constraintCount, out var sourceIndices);
            rawPool.SpecializeFor<int>().Take(constraintCount, out var sortKeys);
            rawPool.SpecializeFor<int>().Take(constraintCount, out var handlesCache);
            rawPool.SpecializeFor<TwoBodyReferences>().Take(bundleCount, out var referencesCache);
            rawPool.SpecializeFor<TPrestepData>().Take(bundleCount, out var prestepCache);
            rawPool.SpecializeFor<TAccumulatedImpulse>().Take(bundleCount, out var accumulatedImpulseCache);

            //Cache the unsorted data.
            BodyReferences.CopyTo(bundleStartIndex, ref referencesCache, 0, bundleCount);
            PrestepData.CopyTo(bundleStartIndex, ref prestepCache, 0, bundleCount);
            AccumulatedImpulses.CopyTo(bundleStartIndex, ref accumulatedImpulseCache, 0, bundleCount);
            IndexToHandle.CopyTo(bundleStartIndex * Vector<int>.Count, ref handlesCache, 0, constraintCount);

            //First, compute the proper order of the constraints in this region by sorting their keys.
            //This minimizes the number of swaps that must be applied to the actual bundle data.
            //Avoiding swaps of actual data is very valuable- a single constraint can be hundreds of bytes, and the accesses to a slot tend to require some complex addressing.
            var baseIndex = bundleStartIndex * Vector<int>.Count;
            for (int i = 0; i < constraintCount; ++i)
            {
                sourceIndices[i] = i;
                sortKeys[i] = GetSortKey(baseIndex + i);
            }
            
            //TODO: You may want to actually change the way the optimizer handles sorting. Rather than doing a bunch of independent sorts, it could do a cooperative sort.
            //It would be slower in terms of raw coverage, but it would converge faster. The key there would be an efficient and parallel merge operation.
            //Subarrays could be sorted by any available means.
            var comparer = default(IntComparer);
            QuickSort.Sort<int, int, Buffer<int>, Buffer<int>, IntComparer>(ref sortKeys, ref sourceIndices, 0, constraintCount - 1, ref comparer);
            
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
            
            rawPool.SpecializeFor<int>().Return(ref sourceIndices);
            rawPool.SpecializeFor<int>().Return(ref sortKeys);
            rawPool.SpecializeFor<int>().Return(ref handlesCache);
            rawPool.SpecializeFor<TwoBodyReferences>().Return(ref referencesCache);
            rawPool.SpecializeFor<TPrestepData>().Return(ref prestepCache);
            rawPool.SpecializeFor<TAccumulatedImpulse>().Return(ref accumulatedImpulseCache);
        }

        public sealed override void ValidateBundleCounts()
        {
            Debug.Assert(constraintCount > 0, "If a type batch exists, it should have constraints in it.");
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
