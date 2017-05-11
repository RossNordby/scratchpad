﻿using BEPUutilities2;
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
        public int GetSortKey(int constraintIndex)
        {
            BundleIndexing.GetBundleIndices(constraintIndex, out var bundleIndex, out var innerIndex);
            ref var bundleReferences = ref BodyReferences[bundleIndex];
            Debug.Assert(constraintIndex < constraintCount);
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

        public override sealed void SortByBodyLocation(int bundleStartIndex, int constraintCount, ConstraintLocation[] handlesToConstraints, int bodyCount, BufferPool rawPool)
        {
            int bundleCount = (constraintCount >> BundleIndexing.VectorShift);
            if ((constraintCount & BundleIndexing.VectorMask) != 0)
                ++bundleCount;

            var intPool = rawPool.SpecializeFor<int>();
            intPool.Take(constraintCount, out var inputSourceIndices);
            intPool.Take(constraintCount, out var sortKeys);
            intPool.Take(constraintCount, out var scratchValues);
            intPool.Take(constraintCount, out var scratchKeys);
            intPool.Take(constraintCount, out var scatterIndices);

            //First we'll compute the proper order of the constraints in this region by sorting their keys.
            //This minimizes the number of swaps that must be applied to the actual bundle data.
            //Avoiding swaps of actual data is very valuable- a single constraint can be hundreds of bytes, and the accesses to a slot tend to require some complex addressing.
            var baseIndex = bundleStartIndex * Vector<int>.Count;
            for (int i = 0; i < constraintCount; ++i)
            {
                inputSourceIndices[i] = i;
                sortKeys[i] = GetSortKey(baseIndex + i);
            }
            //Cache unsorted body references. We do it ahead of time since we already have them in cache from computing the sort keys.
            rawPool.SpecializeFor<TwoBodyReferences>().Take(bundleCount, out var referencesCache);
            BodyReferences.CopyTo(bundleStartIndex, ref referencesCache, 0, bundleCount);

            //We don't bother using multiple threads on the sort. Sorting 4096 elements takes 20 microseconds on a single 4.5ghz 3770K thread.
            //TODO: You could run this alongside parts of the copy operations to hide it, but that's nontrivial complexity for hiding 20us.
            LSBRadixSort.Sort<int, Buffer<int>, Buffer<int>>(ref sortKeys, ref inputSourceIndices, ref scratchKeys, ref scratchValues, 0, constraintCount, bodyCount - 1, rawPool,
                out var sortedKeys, out var sortedSourceIndices);

            //We do this transformation from gather indices to scatter indices on a single thread since the whole thing will likely take less time than a multithread dispatch.
            //Also, it's likely that all the values being gathered are still in L1 cache. If we farmed it out to other threads, that wouldn't be the case.
            for (int i = 0; i < constraintCount; ++i)
            {
                scatterIndices[sortedSourceIndices[i]] = baseIndex + i;
            }

            //Now that we have scatter indices, copy each constraint property into cache and then into final sorted position in sequence.
            //First, scatter body references into position- we already cached them.
            for (int sourceIndex = 0; sourceIndex < constraintCount; ++sourceIndex)
            {
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(scatterIndices[sourceIndex], out var targetBundle, out var targetInner);
                GatherScatter.CopyLane(ref referencesCache[sourceBundle], sourceInner, ref BodyReferences[targetBundle], targetInner);
            }
            rawPool.SpecializeFor<TwoBodyReferences>().Return(ref referencesCache);

#if DEBUG
            var previousKey = -1;
            for (int sourceIndex = 0; sourceIndex < constraintCount; ++sourceIndex)
            {
                var key = GetSortKey(baseIndex + sourceIndex);
                Debug.Assert(key > previousKey, "After the sort and swap completes, all constraints should be in order.");
                Debug.Assert(key == sortedKeys[sourceIndex], "After the swap goes through, the rederived sort keys should match the previously sorted ones.");
                previousKey = key;
            }
#endif
            //Technically these could be returned more aggressively, but we keep them down here for the sake of debugging simplicity.
            intPool.Return(ref inputSourceIndices);
            intPool.Return(ref sortKeys);
            intPool.Return(ref scratchKeys);
            intPool.Return(ref scratchValues);

            //Now cache and scatter the prestep data.
            rawPool.SpecializeFor<TPrestepData>().Take(bundleCount, out var prestepCache);
            PrestepData.CopyTo(bundleStartIndex, ref prestepCache, 0, bundleCount);
            for (int sourceIndex = 0; sourceIndex < constraintCount; ++sourceIndex) //TODO: Try reverse.
            {
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(scatterIndices[sourceIndex], out var targetBundle, out var targetInner);
                GatherScatter.CopyLane(ref prestepCache[sourceBundle], sourceInner, ref PrestepData[targetBundle], targetInner);
            }
            rawPool.SpecializeFor<TPrestepData>().Return(ref prestepCache);

            //Now cache and scatter the accumulated impulse data.
            rawPool.SpecializeFor<TAccumulatedImpulse>().Take(bundleCount, out var accumulatedImpulseCache);
            AccumulatedImpulses.CopyTo(bundleStartIndex, ref accumulatedImpulseCache, 0, bundleCount);
            for (int sourceIndex = 0; sourceIndex < constraintCount; ++sourceIndex) //TODO: Try reverse.
            {
                BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
                BundleIndexing.GetBundleIndices(scatterIndices[sourceIndex], out var targetBundle, out var targetInner);
                GatherScatter.CopyLane(ref accumulatedImpulseCache[sourceBundle], sourceInner, ref AccumulatedImpulses[targetBundle], targetInner);
            }
            rawPool.SpecializeFor<TAccumulatedImpulse>().Return(ref accumulatedImpulseCache);

            //Now cache and scatter the index to constraint index to handle mapping.
            intPool.Take(constraintCount, out var handlesCache);
            IndexToHandle.CopyTo(bundleStartIndex * Vector<int>.Count, ref handlesCache, 0, constraintCount);
            for (int sourceIndex = 0; sourceIndex < constraintCount; ++sourceIndex) //TODO: Try reverse.
            {
                var targetIndex = scatterIndices[sourceIndex];
                var handle = handlesCache[sourceIndex];
                IndexToHandle[targetIndex] = handle;
                //Note that this overwrites some other constraint's entry, but that other constraint will also be overwriting their new spot too.
                handlesToConstraints[handle].IndexInTypeBatch = targetIndex;
            }
            intPool.Return(ref handlesCache);
            intPool.Return(ref scatterIndices);
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
        //                sortKeys[i] = GetSortKey(baseIndex + i);
        //            }

        //            //We don't bother using multiple threads on the sort. Sorting 4096 elements takes 20 microseconds on a single 4.5ghz 3770K thread.
        //            //TODO: You could run this alongside parts of the copy operations to hide it, but that's nontrivial complexity for hiding 20us.
        //            LSBRadixSort.Sort<int, Buffer<int>, Buffer<int>>(ref sortKeys, ref inputSourceIndices, ref scratchKeys, ref scratchValues, 0, constraintCount, bodyCount - 1, rawPool,
        //                out var sortedKeys, out var sortedSourceIndices);

        //            //Push the cached data into its proper sorted position.
        //#if DEBUG
        //                    var previousKey = -1;
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
        //                        var key = GetSortKey(baseIndex + i);
        //                        Debug.Assert(key > previousKey, "After the sort and swap completes, all constraints should be in order.");
        //                        Debug.Assert(key == sortKeys[i], "After the swap goes through, the rederived sort keys should match the previously sorted ones.");
        //                        previousKey = key;
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
