﻿using BEPUutilities2.Collections;
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
        public Vector<int> IndexA;
        public Vector<int> IndexB;

        //Despite having no simd-accelerated shift, it's still a net win to use scalar instructions to extract bundle indices on the fly.
        //By doing the bundle-inner extraction at runtime, we save 8 bytes per constraint. A 6700K supporting AVX2 with 8.75 GBps bandwidth per core can load 
        //the required (8 floats per bundle) * (4 bytes per float) * (2 bodies) = 64 bytes in ~7.3ns.
        //At the base clock rate of 4ghz, that's 29 cycles. On modern processors, many instructions will execute in parallel, so that 7.3ns could be enough time
        //for well over 50 instructions. Considering that all we have to add is 2 SIMD bitwise ANDs and 16 scalar shifts (for AVX2, given current lack of SIMD shift),
        //it's a pretty clear win for any bandwidth constrained use.
        //And, while as of this writing the CoreCLR does not support AVX512, we should expect it someday- and memory bandwidth is going to be even a bigger concern.

        [MethodImpl(MethodImplOptions.NoInlining)]
        public void Unpack(int bundleIndex, int constraintCount, out UnpackedTwoBodyReferences unpacked)
        {
            //Having access to SIMD shift would be really nice. (There's not actually hardware support for int divide, so... per-slot shift it is.)
            ref var bundleA = ref Unsafe.As<Vector<int>, int>(ref unpacked.BundleIndexA);
            ref var bundleB = ref Unsafe.As<Vector<int>, int>(ref unpacked.BundleIndexB);
            ref var indexA = ref Unsafe.As<Vector<int>, int>(ref IndexA);
            ref var indexB = ref Unsafe.As<Vector<int>, int>(ref IndexB);
            //Note that we don't bother using the bundle's count here. 
            //That would only be helpful in one bundle per type batch, so the branching would just be (a tiny amount of) wasted effort almost always.
            Debug.Assert((Vector<int>.Count & 3) == 0, "No current hardware has a non-4-multiple width of 32 bit types, but just in case, note that this requires a width that is a multiple of 4!");

            bundleA = indexA >> BundleIndexing.VectorShift;
            bundleB = indexB >> BundleIndexing.VectorShift;
            Unsafe.Add(ref bundleA, 1) = Unsafe.Add(ref indexA, 1) >> BundleIndexing.VectorShift;
            Unsafe.Add(ref bundleB, 1) = Unsafe.Add(ref indexB, 1) >> BundleIndexing.VectorShift;
            Unsafe.Add(ref bundleA, 2) = Unsafe.Add(ref indexA, 2) >> BundleIndexing.VectorShift;
            Unsafe.Add(ref bundleB, 2) = Unsafe.Add(ref indexB, 2) >> BundleIndexing.VectorShift;
            Unsafe.Add(ref bundleA, 3) = Unsafe.Add(ref indexA, 3) >> BundleIndexing.VectorShift;
            Unsafe.Add(ref bundleB, 3) = Unsafe.Add(ref indexB, 3) >> BundleIndexing.VectorShift;

            for (int i = 4; i < Vector<int>.Count; i += 4)
            {
                ref var targetA = ref Unsafe.Add(ref bundleA, i);
                ref var targetB = ref Unsafe.Add(ref bundleB, i);
                ref var sourceA = ref Unsafe.Add(ref indexA, i);
                ref var sourceB = ref Unsafe.Add(ref indexB, i);
                targetA = sourceA >> BundleIndexing.VectorShift;
                targetB = sourceB >> BundleIndexing.VectorShift;
                Unsafe.Add(ref targetA, 1) = Unsafe.Add(ref sourceA, 1) >> BundleIndexing.VectorShift;
                Unsafe.Add(ref targetB, 1) = Unsafe.Add(ref sourceB, 1) >> BundleIndexing.VectorShift;
                Unsafe.Add(ref targetA, 2) = Unsafe.Add(ref sourceA, 2) >> BundleIndexing.VectorShift;
                Unsafe.Add(ref targetB, 2) = Unsafe.Add(ref sourceB, 2) >> BundleIndexing.VectorShift;
                Unsafe.Add(ref targetA, 3) = Unsafe.Add(ref sourceA, 3) >> BundleIndexing.VectorShift;
                Unsafe.Add(ref targetB, 3) = Unsafe.Add(ref sourceB, 3) >> BundleIndexing.VectorShift;

            }
            
            var mask = new Vector<int>(BundleIndexing.VectorMask);
            unpacked.InnerIndexA = Vector.BitwiseAnd(mask, IndexA);
            unpacked.InnerIndexB = Vector.BitwiseAnd(mask, IndexB);
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

            ref var indexA = ref GatherScatter.Get(ref BodyReferences[constraintBundleIndex].IndexA, constraintInnerIndex);
            ref var indexB = ref Unsafe.Add(ref indexA, Vector<int>.Count);

            //Note that the variables are ref locals! This is important for correctness, because every execution of LoopBody could result in a swap.
            //Ref locals aren't the only solution, but if you ever change this, make sure you account for the potential mutation in the enumerator.
            enumerator.LoopBody(indexA);
            enumerator.LoopBody(indexB);
        }

        public sealed override void UpdateForBodyMemoryMove(int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation)
        {

            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);

            //Note that this relies on the bodyreferences memory layout. It uses the stride of vectors to skip to the next body based on the bodyIndexInConstraint.
            GatherScatter.Get(ref BodyReferences[constraintBundleIndex].IndexA, constraintInnerIndex + bodyIndexInConstraint * Vector<int>.Count) = newBodyLocation;
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
            ref var indexA = ref GatherScatter.Get(ref bundleReferences.IndexA, innerIndex);
            ref var indexB = ref Unsafe.Add(ref indexA, innerIndex);
            return indexA < indexB ? indexA : indexB;
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
