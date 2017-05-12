using BEPUutilities2;
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
        public static int GetSortKey(int constraintIndex, ref Buffer<TwoBodyReferences> bodyReferences)
        {
            BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
            ref var bundleReferences = ref bodyReferences[bundleIndex];
            //We sort based on the body references within the constraint. 
            //Sort based on the smaller body index in a constraint. Note that it is impossible for there to be two references to the same body within a constraint batch, 
            //so there's no need to worry about the case where the comparison is equal.
            ref var indexA = ref GatherScatter.Get(ref bundleReferences.IndexA, innerIndex);
            ref var indexB = ref Unsafe.Add(ref indexA, Vector<int>.Count);
            return indexA < indexB ? indexA : indexB;
            //TODO: It is conceivable that another sorting key heuristic would beat this one. This completely ignores the second connection and makes it very unlikely
            //that it could converge to whatever globally optimal layout exists. It's a little tricky to come up with good heuristics, though- many will end up 
            //batching constraints which relate to wildly different bodies. Sorting by the minimum at least guarantees that two adjacent constraints will be as close as they can be
            //in one way. 

            //In practice, we approach within about 5-10% of the optimum using the above sorting heuristic and the current incremental body optimizer.

            //It's not immediately clear that ANY local comparison based sort will be able to do as well as some form of global optimizer that maximizes
            //the 'closeness', which drops to zero once accesses would leave the cache line. This is made more complicated by the AOSOA layout- most likely
            //such a heuristic would need to score based on whether the bodies are in the same bundle. So, for example, two constraints get one 'close' point for each 
            //shared body bundle.
            //(Since you would only be optimizing within type batches, the fact that different types have different body counts wouldn't be an issue. They would
            //only ever be compared against other constraints of the same type.)
            //Even if you can't guarantee the next constraint is going to have bodies that are in cache, if you can generally lift the number of constraints
            //that end up used quite a few times in L1/L2, there is probably a nice benefit to be had. That would suggest 'wider' optimizations rather than bundle-specific ones.
            //All of these global techniques get into some nasty O complexities, but there may be heuristics which can approach them- sort of like BVH SAH sweep builders.
            //Especially incremental ones, like the refinement we use in the dynamic BVH broadphase.
        }


        internal sealed override void GenerateSortKeysAndCopyReferences(
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref int firstSortKey, ref int firstSourceIndex, ref RawBuffer bodyReferencesCache)
        {
            for (int i = 0; i < constraintCount; ++i)
            {
                Unsafe.Add(ref firstSourceIndex, i) = localConstraintStart + i;
                Unsafe.Add(ref firstSortKey, i) = GetSortKey(constraintStart + i, ref BodyReferences);
            }
            var typedBodyReferencesCache = bodyReferencesCache.As<TwoBodyReferences>();
            BodyReferences.CopyTo(bundleStart, ref typedBodyReferencesCache, localBundleStart, bundleCount);
        }


        internal sealed override void CopyToCache(
            int bundleStart, int localBundleStart, int bundleCount,
            int constraintStart, int localConstraintStart, int constraintCount,
            ref Buffer<int> indexToHandleCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache)
        {
            IndexToHandle.CopyTo(constraintStart, ref indexToHandleCache, 0, constraintCount);
            var typedPrestepCache = prestepCache.As<TPrestepData>();
            var typedAccumulatedImpulsesCache = accumulatedImpulsesCache.As<TAccumulatedImpulse>();
            PrestepData.CopyTo(bundleStart, ref typedPrestepCache, 0, bundleCount);
            AccumulatedImpulses.CopyTo(bundleStart, ref typedAccumulatedImpulsesCache, 0, bundleCount);
        }


        internal sealed override void Regather(int constraintStart, int constraintCount, ref int firstSourceIndex,
            ref Buffer<int> indexToHandleCache, ref RawBuffer bodyReferencesCache, ref RawBuffer prestepCache, ref RawBuffer accumulatedImpulsesCache,
            ConstraintLocation[] handlesToConstraints)
        {
            var typedBodyReferencesCache = bodyReferencesCache.As<TwoBodyReferences>();
            var typedPrestepCache = prestepCache.As<TPrestepData>();
            var typedAccumulatedImpulsesCache = accumulatedImpulsesCache.As<TAccumulatedImpulse>();
            for (int i = 0; i < constraintCount; ++i)
            {
                var sourceIndex = Unsafe.Add(ref firstSourceIndex, i);
                var targetIndex = constraintStart + i;
                //Note that we do not bother checking whether the source and target are the same.
                //The cost of the branch is large enough in comparison to the frequency of its usefulness that it only helps in practically static situations.
                //Also, its maximum benefit is quite small.
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

                Move(ref typedBodyReferencesCache[sourceBundle], ref typedPrestepCache[sourceBundle], ref typedAccumulatedImpulsesCache[sourceBundle],
                    sourceInner, indexToHandleCache[sourceIndex],
                    targetBundle, targetInner, targetIndex, handlesToConstraints);

            }
        }

        internal override sealed void VerifySortRegion(int bundleStartIndex, int constraintCount, ref Buffer<int> sortedKeys, ref Buffer<int> sortedSourceIndices)
        {
            var previousKey = -1;
            var baseIndex = bundleStartIndex << BundleIndexing.VectorShift;
            for (int i = 0; i < constraintCount; ++i)
            {
                var sourceIndex = sortedSourceIndices[i];
                var targetIndex = baseIndex + i;
                var key = GetSortKey(baseIndex + i, ref BodyReferences);
                Debug.Assert(key > previousKey, "After the sort and swap completes, all constraints should be in order.");
                Debug.Assert(key == sortedKeys[i], "After the swap goes through, the rederived sort keys should match the previously sorted ones.");
                previousKey = key;

            }
        }

        //        public override sealed void SortByBodyLocation(int bundleStartIndex, int constraintCount, ConstraintLocation[] handlesToConstraints, int bodyCount, BufferPool rawPool)
        //        {
        //            int bundleCount = (constraintCount >> BundleIndexing.VectorShift);
        //            if ((constraintCount & BundleIndexing.VectorMask) != 0)
        //                ++bundleCount;

        //            var intPool = rawPool.SpecializeFor<int>();
        //            intPool.Take(constraintCount, out var inputSourceIndices);
        //            intPool.Take(constraintCount, out var sortKeys);
        //            intPool.Take(constraintCount, out var scratchValues);
        //            intPool.Take(constraintCount, out var scratchKeys);
        //            intPool.Take(constraintCount, out var handlesCache);
        //            rawPool.SpecializeFor<TwoBodyReferences>().Take(bundleCount, out var referencesCache);
        //            rawPool.SpecializeFor<TPrestepData>().Take(bundleCount, out var prestepCache);
        //            rawPool.SpecializeFor<TAccumulatedImpulse>().Take(bundleCount, out var accumulatedImpulseCache);

        //            //Cache the unsorted data.
        //            BodyReferences.CopyTo(bundleStartIndex, ref referencesCache, 0, bundleCount);
        //            PrestepData.CopyTo(bundleStartIndex, ref prestepCache, 0, bundleCount);
        //            AccumulatedImpulses.CopyTo(bundleStartIndex, ref accumulatedImpulseCache, 0, bundleCount);
        //            IndexToHandle.CopyTo(bundleStartIndex * Vector<int>.Count, ref handlesCache, 0, constraintCount);

        //            //First, compute the proper order of the constraints in this region by sorting their keys.
        //            //This minimizes the number of swaps that must be applied to the actual bundle data.
        //            //Avoiding swaps of actual data is very valuable- a single constraint can be hundreds of bytes, and the accesses to a slot tend to require some complex addressing.
        //            var baseIndex = bundleStartIndex * Vector<int>.Count;
        //            for (int i = 0; i < constraintCount; ++i)
        //            {
        //                inputSourceIndices[i] = i;
        //                sortKeys[i] = GetSortKey(baseIndex + i, ref BodyReferences);
        //            }

        //            LSBRadixSort.Sort<int, Buffer<int>, Buffer<int>>(ref sortKeys, ref inputSourceIndices, ref scratchKeys, ref scratchValues, 0, constraintCount, bodyCount - 1, rawPool,
        //                out var sortedKeys, out var sortedSourceIndices);

        //            //Push the cached data into its proper sorted position.
        //#if DEBUG
        //            var previousKey = -1;
        //#endif
        //            for (int i = 0; i < constraintCount; ++i)
        //            {
        //                var sourceIndex = sortedSourceIndices[i];
        //                var targetIndex = baseIndex + i;
        //                //Note that we do not bother checking whether the source and target are the same.
        //                //The cost of the branch is large enough in comparison to the frequency of its usefulness that it only helps in practically static situations.
        //                //Also, its maximum benefit is quite small.
        //                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
        //                BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);

        //                Move(ref referencesCache[sourceBundle], ref prestepCache[sourceBundle], ref accumulatedImpulseCache[sourceBundle],
        //                    sourceInner, handlesCache[sourceIndex],
        //                    targetBundle, targetInner, targetIndex, handlesToConstraints);

        //#if DEBUG
        //                var key = GetSortKey(baseIndex + i);
        //                Debug.Assert(key > previousKey, "After the sort and swap completes, all constraints should be in order.");
        //                Debug.Assert(key == sortedKeys[i], "After the swap goes through, the rederived sort keys should match the previously sorted ones.");
        //                previousKey = key;
        //#endif
        //            }

        //            intPool.Return(ref inputSourceIndices);
        //            intPool.Return(ref sortKeys);
        //            intPool.Return(ref scratchKeys);
        //            intPool.Return(ref scratchValues);
        //            intPool.Return(ref handlesCache);
        //            rawPool.SpecializeFor<TwoBodyReferences>().Return(ref referencesCache);
        //            rawPool.SpecializeFor<TPrestepData>().Return(ref prestepCache);
        //            rawPool.SpecializeFor<TAccumulatedImpulse>().Return(ref accumulatedImpulseCache);
        //        }

    }
}
