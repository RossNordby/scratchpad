using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace SolverPrototype.CollisionDetection
{

    public enum ConstraintGeneratorType
    {
        /// <summary>
        /// Pair which will directly produce constraints.
        /// </summary>
        Direct = 0,
        /// <summary>
        /// Pair expecting both a discrete and inner sphere manifolds.
        /// </summary>
        Linear = 1,
        /// <summary>
        /// Pair expecting multiple discrete manifolds.
        /// </summary>
        Substep = 2,
        /// <summary>
        /// Pair expecting both inner sphere manifolds and multiple discrete manifolds.
        /// </summary>
        SubstepWithLinear = 3
    }

    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        public struct ConstraintGenerators : IContinuations
        {
            int workerIndex;
            BufferPool pool;
            NarrowPhase<TCallbacks> narrowPhase;

            unsafe static void ResolveLinearManifold(ContactManifold* mainManifold, ContactManifold* linearManifold, ContactManifold* outManifold)
            {
                var linearCount = linearManifold->ContactCount;
                if (linearCount > 0)
                {
                    Debug.Assert(linearCount <= 2, "Inner sphere derived contacts should only ever contribute one contact per involved body.");

                    var mainCount = mainManifold->ContactCount;
                    if (mainCount > 0)
                    {
                        var totalCount = mainCount + linearCount;
                        var contactsToPotentiallyReplaceCount = totalCount - 4;
                        if (contactsToPotentiallyReplaceCount > 0)
                        {
                            //There are more contacts than slots. A subset must be prioritized.
                            //We want the deepest set of contacts from all available manifolds. 
                            //(This isn't necessarily an ideal heuristic- consider a bunch of deep  contacts all in the same place,
                            //causing the removal of a distant but less redundant contact. In practice, though, it works okay.)

                            //To find the deepest contacts, simply sort both sets and pop the minimums.
                            //Rather than shuffling the contact manifold memory around, just sort indices.

                            //TODO: This entire hardcoded sort is a bit gross and silly. You could do better.
                            //TODO: It's not clear that this is actually even useful. It may be that just picking the deepest of the linear contacts and filling any open space
                            //is the best and simplest option in the end.
                            var linearIndices = stackalloc int[linearCount];
                            var mainIndices = stackalloc int[mainCount];
                            if (linearCount == 2)
                            {
                                if (linearManifold->Depth0 < linearManifold->Depth1)
                                {
                                    linearIndices[0] = 0;
                                    linearIndices[1] = 1;
                                }
                                else
                                {
                                    linearIndices[0] = 0;
                                    linearIndices[1] = 1;
                                }
                            }
                            else
                            {
                                linearIndices[0] = 0;
                            }

                            for (int i = 0; i < mainCount; ++i)
                                mainIndices[i] = i;

                            outManifold->SetConvexityAndCount(totalCount, false);
                            var mainDepths = &mainManifold->Depth0;
                            for (int i = 1; i <= mainCount; ++i)
                            {
                                var originalIndex = mainIndices[i];
                                var depth = mainDepths[originalIndex];
                                int compareIndex;
                                for (compareIndex = i - 1; compareIndex >= 0; --compareIndex)
                                {
                                    var compareDepth = mainDepths[compareIndex];
                                    if (compareDepth < depth)
                                    {
                                        //Move the element up one slot.
                                        var upperSlotIndex = compareIndex + 1;
                                        mainIndices[upperSlotIndex] = mainIndices[compareIndex];
                                    }
                                    else
                                        break;
                                }
                                var targetIndex = compareIndex + 1;
                                if (targetIndex != i)
                                {
                                    //Move the original index down.
                                    mainIndices[targetIndex] = originalIndex;
                                }
                            }

                            var outIndex = 0;
                            var mainIndex = 0;
                            var linearIndex = 0;
                            var outOffsets = &outManifold->Offset0;
                            var outDepths = &outManifold->Depth0;
                            var outBases = &outManifold->Normal0;
                            var outIds = &outManifold->FeatureId0;
                            var linearOffsets = &linearManifold->Offset0;
                            var linearDepths = &linearManifold->Depth0;
                            var linearBases = &linearManifold->Normal0;
                            var linearIds = &linearManifold->FeatureId0;
                            var mainOffsets = &mainManifold->Offset0;
                            var mainIds = &mainManifold->FeatureId0;
                            if (mainManifold->Convex)
                            {
                                //While the linear and output manifolds are nonconvex, the main one is convex.
                                while (outIndex < 4)
                                {
                                    if (linearIndex == linearCount ||
                                        (mainIndex < mainCount &&
                                        mainDepths[mainIndex] > linearDepths[linearIndex]))
                                    {
                                        outOffsets[outIndex] = mainOffsets[mainIndex];
                                        outDepths[outIndex] = mainDepths[mainIndex];
                                        outBases[outIndex] = mainManifold->ConvexNormal;
                                        outIds[outIndex] = mainIds[mainIndex];
                                        ++mainIndex;
                                    }
                                    else
                                    {
                                        outOffsets[outIndex] = linearOffsets[linearIndex];
                                        outDepths[outIndex] = linearDepths[linearIndex];
                                        outBases[outIndex] = linearBases[linearIndex];
                                        outIds[outIndex] = linearIds[linearIndex];
                                        ++linearIndex;
                                    }
                                }
                            }
                            else
                            {
                                //Both manifolds are nonconvex.
                                var mainBases = &mainManifold->Normal0;
                                while (outIndex < 4)
                                {
                                    if (linearIndex == linearCount ||
                                        (mainIndex < mainCount &&
                                        mainDepths[mainIndex] > linearDepths[linearIndex]))
                                    {
                                        outOffsets[outIndex] = mainOffsets[mainIndex];
                                        outDepths[outIndex] = mainDepths[mainIndex];
                                        outBases[outIndex] = mainBases[mainIndex];
                                        outIds[outIndex] = mainIds[mainIndex];
                                        ++mainIndex;
                                    }
                                    else
                                    {
                                        outOffsets[outIndex] = linearOffsets[linearIndex];
                                        outDepths[outIndex] = linearDepths[linearIndex];
                                        outBases[outIndex] = linearBases[linearIndex];
                                        outIds[outIndex] = linearIds[linearIndex];
                                        ++linearIndex;
                                    }
                                    ++outIndex;
                                }
                            }
                        }
                        else
                        {
                            //There is sufficient room in the manifold to include all the new contacts, so there is no need for prioritization.
                            outManifold->SetConvexityAndCount(totalCount, false);
                            //Add all existing contacts. Note that the use of inner sphere contacts forces the manifold to be nonconvex unconditionally.
                            //While there are cases in which the normals could actually be planar, we don't spend the time figuring that out-
                            //this state will be extremely brief regardless, and there isn't much value in trying to tease out convexity for one or two frames.
                            var outOffsets = &outManifold->Offset0;
                            var outDepths = &outManifold->Depth0;
                            var outBases = &outManifold->Normal0;
                            var outIds = &outManifold->FeatureId0;
                            var mainOffsets = &mainManifold->Offset0;
                            var mainDepths = &mainManifold->Depth0;
                            var mainIds = &mainManifold->FeatureId0;
                            if (mainManifold->Convex)
                            {
                                for (int i = 0; i < mainCount; ++i)
                                {
                                    outOffsets[i] = mainOffsets[i];
                                    outDepths[i] = mainDepths[i];
                                    outBases[i] = mainManifold->ConvexNormal;
                                    outIds[i] = mainIds[i];
                                }
                            }
                            else
                            {
                                var mainBases = &mainManifold->Normal0;
                                for (int i = 0; i < mainCount; ++i)
                                {
                                    outOffsets[i] = mainOffsets[i];
                                    outDepths[i] = mainDepths[i];
                                    outBases[i] = mainBases[i];
                                    outIds[i] = mainIds[i];
                                }
                            }
                            //Now add the linear contacts. Both manifolds are known to be nonconvex. 
                            var linearOffsets = &linearManifold->Offset0;
                            var linearDepths = &linearManifold->Depth0;
                            var linearBases = &linearManifold->Normal0;
                            var linearIds = &linearManifold->FeatureId0;
                            for (int linearIndex = 0; linearIndex < linearCount; ++linearIndex)
                            {
                                var outIndex = mainCount + linearIndex;
                                outOffsets[outIndex] = linearOffsets[linearIndex];
                                outDepths[outIndex] = linearDepths[linearIndex];
                                outBases[outIndex] = linearBases[linearIndex];
                                outIds[outIndex] = linearIds[linearIndex];
                            }
                        }
                    }
                    else
                    {
                        outManifold = linearManifold;
                    }
                }
                else
                {
                    outManifold = mainManifold;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            unsafe static void ResolveSubstepManifold(ref SubstepManifolds substeps, ContactManifold* outManifold)
            {
                //Scan the substeps looking for the first substep that contains any contacts.
                //TODO: There are situations involving very high angular velocity where the first contacts are not the best choice. 
                //If this turns out to be a problem in practice, you may want to change the heuristic to prefer *approaching* contacts over merely existing contacts.
                //However, determining whether a contact is approaching requires computing the relative velocity at its position, which isn't free. (Not super expensive, but not free.)
                //For the sake of simplicity, just use 'first contact' for now.

                //TODO: If the first manifold is not full, you could also pull contacts from later substeps. They might help post-collision rotate-through-the-ground type penetration.

                float inverseCount = 1f / substeps.Manifolds.Count;
                for (int i = 0; i < substeps.Manifolds.Count; ++i)
                {
                    ref var manifold = ref substeps.Manifolds[i];
                    var contactCount = manifold.ContactCount;
                    if (contactCount > 0)
                    {
                        *outManifold = manifold;
                        //Once the best substep is selected, transform the contact positions and depths to be relative to the poses at t=0.
                        //Since the contact position offsets are not rotated, all we have to do is add the offset from t=0 to the current time to each contact position
                        //and modify the penetration depths according to that offset along the normal.
                        var offset = substeps.RelativeOffsetChange * (i * inverseCount);
                        var offsets = &outManifold->Offset0;
                        var depths = &outManifold->Depth0;
                        //TODO: these two TransformY's could be optimized with knowledge that it's unit length. 
                        //That would be pretty useful generally- I'm not sure we've ever used those functions without it being unit length.
                        //Pretty micro-optimizey, though.
                        if (outManifold->Convex)
                        {
                            var penetrationOffset = Vector3.Dot(offset, outManifold->ConvexNormal);
                            for (int j = 0; j < contactCount; ++j)
                            {
                                offsets[j] += offset;
                                depths[j] += penetrationOffset;
                            }
                        }
                        else
                        {
                            var normals = &outManifold->Normal0;
                            for (int j = 0; j < contactCount; ++j)
                            {
                                var penetrationOffset = Vector3.Dot(offset, normals[j]);
                                offsets[j] += offset;
                                depths[j] += penetrationOffset;
                            }
                        }
                        return;
                    }
                }
                //If there are no contacts, then just return an empty manifold.
                *outManifold = substeps.Manifolds[0];
            }

            struct SubstepManifolds
            {
                public QuickList<ContactManifold, Buffer<ContactManifold>> Manifolds;
                public Vector3 RelativeOffsetChange;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int capacity)
                {
                    QuickList<ContactManifold, Buffer<ContactManifold>>.Create(pool.SpecializeFor<ContactManifold>(), capacity, out Manifolds);
                }


            }

            //Discrete pairs are simply stored as collidable pairs; there is no additional cached data.
            struct LinearPair
            {
                public ContactManifold DiscreteManifold;
                public ContactManifold LinearManifold;
                public CollidablePair Pair;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(CollidablePair pair)
                {
                    Pair = pair;
                    ManifoldsReported = 0;
                    LinearManifold.SetConvexityAndCount(2, false);
                }
            }

            struct SubstepPair
            {
                public SubstepManifolds Manifolds;
                public CollidablePair Pair;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int substepCapacity, CollidablePair pair)
                {
                    Manifolds.Initialize(pool, substepCapacity);
                    Pair = pair;
                    ManifoldsReported = 0;
                }
            }

            struct SubstepWithLinearPair
            {
                public SubstepManifolds SubstepManifolds;
                public ContactManifold LinearManifold;
                public CollidablePair Pair;
                public int ManifoldsReported;

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Initialize(BufferPool pool, int substepCapacity, CollidablePair pair)
                {
                    SubstepManifolds.Initialize(pool, substepCapacity);
                    Pair = pair;
                    ManifoldsReported = 0;
                    LinearManifold.SetConvexityAndCount(2, false);
                }
            }


            struct ContinuationCache<T>
            {
                public IdPool<Buffer<int>> Ids;
                public Buffer<T> Caches;

                public ContinuationCache(BufferPool pool)
                {
                    IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), 32, out Ids);
                    pool.SpecializeFor<T>().Take(128, out Caches);
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public ref T Allocate(BufferPool pool, out int index)
                {
                    index = Ids.Take();
                    if (Caches.Length < index)
                    {
                        pool.SpecializeFor<T>().Resize(ref Caches, index, Caches.Length);
                    }
                    return ref Caches[index];
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Return(int index, BufferPool pool)
                {
                    Ids.Return(index, pool.SpecializeFor<int>());
                }

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public void Dispose(BufferPool pool)
                {
                    Ids.Dispose(pool.SpecializeFor<int>());
                    pool.SpecializeFor<T>().Return(ref Caches);
                }
            }

            ContinuationCache<CollidablePair> discrete;
            ContinuationCache<LinearPair> linear;
            ContinuationCache<SubstepPair> substep;
            ContinuationCache<SubstepWithLinearPair> substepWithLinear;

            public ConstraintGenerators(int workerIndex, BufferPool pool, NarrowPhase<TCallbacks> narrowPhase)
            {
                this.pool = pool;
                this.workerIndex = workerIndex;
                this.narrowPhase = narrowPhase;
                discrete = new ContinuationCache<CollidablePair>(pool);
                linear = new ContinuationCache<LinearPair>(pool);
                substep = new ContinuationCache<SubstepPair>(pool);
                substepWithLinear = new ContinuationCache<SubstepWithLinearPair>(pool);
            }

            static class CCDFeatureIdOffsets
            {
                public const int LinearA = 1 << 16;
                public const int LinearB = 1 << 17;
                //Substeps are simply a series of discrete steps, so you don't want to offset them and make actual discrete contacts unshared.
            }



            private unsafe void FillLinearManifoldSlotA(ref ContactManifold linearManifold, ContactManifold* manifold)
            {
                //Note that linear A is always in slot 0, and linear B is always in slot 1.
                //This is allowed because a linear pair is guaranteed to have two contacts generated from CCD.
                //There is no separation limit on inner sphere pairs, and you never just create one sphere-collidable pair- it's always bilateral.
                //Also note that offsetB is only used from linear A, not linear B. They should be identical.                       
                Debug.Assert(manifold->ContactCount == 1);
                linearManifold.OffsetB = manifold->OffsetB;
                linearManifold.Offset0 = manifold->Offset0;
                linearManifold.Depth0 = manifold->Depth0;
                linearManifold.FeatureId0 = CCDFeatureIdOffsets.LinearA;
                linearManifold.Normal0 = manifold->Normal0;

            }
            private unsafe void FillLinearManifoldSlotB(ref ContactManifold linearManifold, ContactManifold* manifold)
            {
                Debug.Assert(manifold->ContactCount == 1);
                linearManifold.Offset1 = manifold->Offset0;
                linearManifold.Depth1 = manifold->Depth0;
                linearManifold.FeatureId1 = CCDFeatureIdOffsets.LinearB;
                linearManifold.Normal1 = manifold->Normal0;
            }


            public unsafe void Notify(ContinuationIndex continuationId, ContactManifold* manifold)
            {
                var todoTestCollisionCache = default(EmptyCollisionCache);
                var continuationIndex = continuationId.Index;
                switch ((ConstraintGeneratorType)continuationId.Type)
                {
                    case ConstraintGeneratorType.Direct:
                        {
                            //Direct has no need for accumulating multiple reports; we can immediately dispatch.
                            ref var continuation = ref discrete.Caches[continuationIndex];
                            narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation, manifold, ref todoTestCollisionCache);
                            discrete.Return(continuationIndex, pool);
                        }
                        break;
                    //TODO: Note that we could avoid a copy on all the multi-manifold continuations-
                    //rather than creating a manifold and passing it in, we could let the user 'allocate' from the continuation
                    //and then fill the appropriate slot. However, this complicates the api- the user would then have to say 'okay im done' to trigger the flush.
                    case ConstraintGeneratorType.Linear:
                        {
                            ref var continuation = ref linear.Caches[continuationIndex];
                            Debug.Assert(continuation.ManifoldsReported < 2 || manifold->OffsetB == continuation.LinearManifold.OffsetB,
                                "The offset from A to B should be the same in both LinearA and linearB. This requires a guarantee on the part of work submission; did you break that?");
                            switch (continuationId.InnerIndex)
                            {
                                case 0:
                                    //Discrete
                                    continuation.DiscreteManifold = *manifold;
                                    break;
                                case 1:
                                    FillLinearManifoldSlotA(ref continuation.LinearManifold, manifold);
                                    break;
                                case 2:
                                    FillLinearManifoldSlotB(ref continuation.LinearManifold, manifold);
                                    break;
                            }
                            ++continuation.ManifoldsReported;

                            if (continuation.ManifoldsReported == 3)
                            {
                                var manifolds = (ContactManifold*)Unsafe.AsPointer(ref continuation.DiscreteManifold);
                                ContactManifold combinedManifold;
                                ResolveLinearManifold(manifolds, manifolds + 1, &combinedManifold);
                                narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, &combinedManifold, ref todoTestCollisionCache);
                                linear.Return(continuationIndex, pool);
                            }
                        }
                        break;
                    case ConstraintGeneratorType.Substep:
                        {
                            ref var continuation = ref substep.Caches[continuationId.Index];
                            Debug.Assert(continuationId.InnerIndex >= 0 && continuationId.InnerIndex < continuation.Manifolds.Manifolds.Count);
                            continuation.Manifolds.Manifolds[continuationId.InnerIndex] = *manifold;
                            ++continuation.ManifoldsReported;

                            if (continuation.ManifoldsReported == continuation.Manifolds.Manifolds.Count)
                            {
                                ContactManifold resolvedManifold;
                                ResolveSubstepManifold(ref continuation.Manifolds, &resolvedManifold);
                                narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, &resolvedManifold, ref todoTestCollisionCache);
                                substep.Return(continuationIndex, pool);
                            }
                        }
                        break;
                    case ConstraintGeneratorType.SubstepWithLinear:
                        {
                            ref var continuation = ref this.substepWithLinear.Caches[continuationId.Index];
                            var innerIndex = continuationId.InnerIndex;
                            switch (innerIndex)
                                                            {
                                case 0:
                                    FillLinearManifoldSlotA(ref continuation.LinearManifold, manifold);
                                    break;
                                case 1:
                                    FillLinearManifoldSlotB(ref continuation.LinearManifold, manifold);
                                    break;
                                default:
                                    var substepIndex = innerIndex - 2;
                                    Debug.Assert(substepIndex >= 0 && substepIndex < continuation.SubstepManifolds.Manifolds.Count);
                                    continuation.SubstepManifolds.Manifolds[substepIndex] = *manifold;
                                    break;
                            }
                            ++continuation.ManifoldsReported;

                            if (continuation.ManifoldsReported == continuation.SubstepManifolds.Manifolds.Count + 2)
                            {
                                ContactManifold substepManifold, completeManifold;
                                ResolveSubstepManifold(ref continuation.SubstepManifolds, &substepManifold);
                                ResolveLinearManifold(&substepManifold, (ContactManifold*)Unsafe.AsPointer(ref continuation.LinearManifold), &completeManifold);
                                narrowPhase.UpdateConstraintsForPair(workerIndex, ref continuation.Pair, &completeManifold, ref todoTestCollisionCache);
                                substepWithLinear.Return(continuationIndex, pool);
                            }
                        }
                        break;
                }

            }
        }

        ConstraintGenerators[] constraintGenerators;

        private void PrepareConstraintGenerators(IThreadDispatcher threadDispatcher)
        {
            var threadCount = threadDispatcher == null ? 1 : threadDispatcher.ThreadCount;
            //Resizes should be very rare, and having a single extra very small array isn't concerning.
            //(It's not an unmanaged type because it contains nonblittable references.)
            if (constraintGenerators == null || constraintGenerators.Length < threadCount)
                Array.Resize(ref constraintGenerators, threadCount);
            for (int i = 0; i < threadCount; ++i)
            {
                constraintGenerators[i] = new ConstraintGenerators(i, threadDispatcher != null ? threadDispatcher.GetThreadMemoryPool(i) : Pool, this);
            }
        }


    }


}
