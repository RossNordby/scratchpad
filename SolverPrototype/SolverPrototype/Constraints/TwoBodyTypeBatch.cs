using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{
    /// <summary>
    /// A constraint's body references. Stored separately from the iteration data since it is accessed by both the prestep and solve.
    /// Two address streams isn't much of a problem for prefetching.
    /// </summary>
    public struct TwoBodyReferences
    {
        //Unfortunately, there does not exist any Vector<int>.Shift instruction yet, so we cannot efficiently derive the bundle and inner indices from the 'true' indices on the fly.
        //Instead, group references are preconstructed and cached in a nonvectorized way.
        public Vector<int> BundleIndexA;
        public Vector<int> InnerIndexA;
        public Vector<int> BundleIndexB;
        public Vector<int> InnerIndexB;
        //TODO: despite having no simd-accelerated shift, it may still be a net win to use scalar instructions to extract bundle/inner.
        //By combining, we save 8 bytes per constraint. In a bandwidth constrained case, a 6700K supporting AVX2 with 8.75GBps bandwidth available per core 
        //can load the needed 8 bytes in about a nanosecond. We add a bitwise AND and shift. Two bitwise ands and shifts will likely take less than a nanosecond on a 6700K or any other
        //modern system. In fact, the throughput on ANDs on skylake is up to four instructions per cycle. And there are about 4 cycles in a nanosecond.
        //So, even though you'd have to do 16 ands and 16 shifts for AVX512, that would still very likely take less than 16 cycles (~4 nanoseconds).
        //At 8.75GBps, the resulting savings of 8 * 4 * 2 = 64 bytes would have otherwise taken ~7 nanoseconds. So it's a net win for multithreaded use.
        //Notably, AVX512 systems will tend to have higher memory bandwidth. It's not yet clear what will happen for consumer tier parts, but it's likely that a similar bandwidth bump will occur.
        //If we ever get a Vector.Shift, there is no doubt that recomputing the inner index is the right choice.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void Unpack(int bundleIndex, int constraintCount, out UnpackedTwoBodyReferences unpacked)
        {
            unpacked.BundleIndexA = BundleIndexA;
            unpacked.InnerIndexA = InnerIndexA;
            unpacked.BundleIndexB = BundleIndexB;
            unpacked.InnerIndexB = InnerIndexB;
            unpacked.Count = bundleIndex == (constraintCount >> BundleIndexing.VectorShift) ? constraintCount & BundleIndexing.VectorMask : Vector<float>.Count;
        }
    }

    /// <summary>
    /// The rehydrated version of the TwoBodyReferences used during solving.
    /// </summary>
    public struct UnpackedTwoBodyReferences
    {
        public Vector<int> BundleIndexA;
        public Vector<int> InnerIndexA;
        public Vector<int> BundleIndexB;
        public Vector<int> InnerIndexB;
        public int Count;
    }

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
            //This is a little defensive; in the event that the body set actually scales down, you don't want to end up with invalid pointers in some lanes.
            //TODO: This may need to change depending on how we handle kinematic/static/inactive storage and encoding.
            GatherScatter.ClearLane<TwoBodyReferences, int>(ref bundle, innerIndex);

        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        int GetSortKey(int constraintIndex)
        {
            BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
            ref var bundleReferences = ref BodyReferences[bundleIndex];
            Debug.Assert(constraintIndex < constraintCount);
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
            QuickSort.Sort(ref sortKeys[0], ref sourceIndices[0], 0, constraintCount - 1, ref comparer);

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

    }
}
