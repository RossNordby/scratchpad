﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System.Runtime.CompilerServices;
using SolverPrototype.Constraints;
using System.Diagnostics;
using System.Numerics;
using SolverPrototype.Collidables;
using System;


namespace SolverPrototype.CollisionDetection
{
    public interface IPendingContactConstraint
    {
        int PendingBatchId { get; }
    }

    public struct PendingTwoBodyConvex4Constraint : IPendingContactConstraint
    {
        public CollidableReference A;
        public CollidableReference B;
        public ContactManifold4Constraint Description;
        public float AccumulatedImpulse0;
        public float AccumulatedImpulse1;
        public float AccumulatedImpulse2;
        public float AccumulatedImpulse3;

        public int PendingBatchId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return 3; } }
    }


    /// <summary>
    /// Associated with a pair of two collidables that each are controlled by bodies.
    /// </summary>
    struct TwoBodyHandles
    {
        public int A;
        public int B;
    }

    /// <summary>
    /// Special type for collision pairs that do not need to store any supplementary information.
    /// </summary>
    struct EmptyCollisionCache : IPairCacheEntry
    {
        public int TypeId => -1;
    }

    public struct ContactImpulses
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
        public float Impulse3;
    }

    public partial class NarrowPhase<TCallbacks> where TCallbacks : struct, INarrowPhaseCallbacks
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void RedistributeImpulses(int oldContactCount, float* oldImpulses, int* oldFeatureIds, ContactManifold* manifold, float* newImpulses)
        {
            //Map the new contacts to the old contacts.
            var newFeatureIds = &manifold->FeatureId0;
            var newContactCount = manifold->ContactCount;
            for (int i = 0; i < newContactCount; ++i)
            {
                newImpulses[i] = 0;
                for (int j = 0; j < oldContactCount; ++j)
                {
                    if (oldFeatureIds[j] == newFeatureIds[i])
                    {
                        newImpulses[i] = oldImpulses[j];
                    }
                }
            }
            //TODO: 'Unclaimed' impulse from old unmatched contacts could be redistributed to try to conserve total impulse. Something to fiddle with once we have a test case running.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void RequestAddConstraint<TDescription, TBodyHandles>(int workerIndex, PairCacheIndex constraintCacheIndex, ContactImpulses* newImpulses,
           ref TDescription description, TBodyHandles bodyHandles) where TDescription : IConstraintDescription<TDescription>
        {
            if (typeof(TBodyHandles) == typeof(int))
            {
                //This is a single body constraint.
                Callbacks.AddConstraint(workerIndex, constraintCacheIndex, ref *newImpulses, Unsafe.As<TBodyHandles, int>(ref bodyHandles), ref description);
            }
            else if (typeof(TBodyHandles) == typeof(TwoBodyHandles))
            {
                //Two body constraint.
                var typedBodyHandles = Unsafe.As<TBodyHandles, TwoBodyHandles>(ref bodyHandles);
                Callbacks.AddConstraint(workerIndex, constraintCacheIndex, ref *newImpulses, typedBodyHandles.A, typedBodyHandles.B, ref description);
            }
            else
            {
                throw new InvalidOperationException("Invalid body handles type.");
            }
        }


        unsafe void UpdateConstraint<TDescription, TCollisionCache, TConstraintCache, TBodyHandles>(int workerIndex, ref CollidablePair pair,
            ContactManifold* manifold, int manifoldTypeAsConstraintType, ref TCollisionCache collisionCache, ref TDescription description, TBodyHandles bodyHandles)
            where TConstraintCache : IPairCacheEntry
            where TCollisionCache : IPairCacheEntry
            where TDescription : IConstraintDescription<TDescription>
        {
            var index = PairCache.IndexOf(ref pair);
            if (index >= 0)
            {
                //The previous frame had a constraint for this pair.
                ref var pointers = ref PairCache.GetPointers(index);
                Debug.Assert(pointers.ConstraintCache.Exists, "If a pair was persisted in the narrow phase, there should be a constraint associated with it.");
                //TODO: Mark the entry as non-stale. 

                var constraintCacheIndex = pointers.ConstraintCache;
                //TODO: Check codegen; this if statement should JIT to a single path.
                ConstraintReference constraintReference;
                int constraintHandle;
                ContactImpulses newImpulses;
                if (typeof(TConstraintCache) == typeof(ConstraintCache1))
                {
                    ref var constraintCache = ref PairCache.GetConstraintCache<ConstraintCache1>(constraintCacheIndex);
                    constraintHandle = constraintCache.ConstraintHandle;
                    Solver.GetConstraintReference(constraintCache.ConstraintHandle, out constraintReference);
                    //If there's only one contact, assume we can directly reuse the old impulse even when the feature id wouldn't match.
                    PairCache.GatherOldImpulses(constraintCacheIndex.Type, ref constraintReference, &newImpulses.Impulse0);
                }
                else if (typeof(TConstraintCache) == typeof(ConstraintCache2))
                {
                    const int oldContactCount = 2;
                    ref var constraintCache = ref PairCache.GetConstraintCache<ConstraintCache2>(constraintCacheIndex);
                    constraintHandle = constraintCache.ConstraintHandle;
                    Solver.GetConstraintReference(constraintCache.ConstraintHandle, out constraintReference);
                    var oldImpulses = stackalloc float[oldContactCount];
                    PairCache.GatherOldImpulses(constraintCacheIndex.Type, ref constraintReference, oldImpulses);
                    RedistributeImpulses(oldContactCount, oldImpulses, (int*)Unsafe.AsPointer(ref constraintCache.FeatureId0), manifold, &newImpulses.Impulse0);
                }
                else if (typeof(TConstraintCache) == typeof(ConstraintCache3))
                {
                    const int oldContactCount = 3;
                    ref var constraintCache = ref PairCache.GetConstraintCache<ConstraintCache3>(constraintCacheIndex);
                    constraintHandle = constraintCache.ConstraintHandle;
                    Solver.GetConstraintReference(constraintCache.ConstraintHandle, out constraintReference);
                    var oldImpulses = stackalloc float[oldContactCount];
                    PairCache.GatherOldImpulses(constraintCacheIndex.Type, ref constraintReference, oldImpulses);
                    RedistributeImpulses(oldContactCount, oldImpulses, (int*)Unsafe.AsPointer(ref constraintCache.FeatureId0), manifold, &newImpulses.Impulse0);
                }
                else if (typeof(TConstraintCache) == typeof(ConstraintCache4))
                {
                    const int oldContactCount = 4;
                    ref var constraintCache = ref PairCache.GetConstraintCache<ConstraintCache4>(constraintCacheIndex);
                    constraintHandle = constraintCache.ConstraintHandle;
                    Solver.GetConstraintReference(constraintCache.ConstraintHandle, out constraintReference);
                    var oldImpulses = stackalloc float[oldContactCount];
                    PairCache.GatherOldImpulses(constraintCacheIndex.Type, ref constraintReference, oldImpulses);
                    RedistributeImpulses(oldContactCount, oldImpulses, (int*)Unsafe.AsPointer(ref constraintCache.FeatureId0), manifold, &newImpulses.Impulse0);
                }
                else
                {
                    throw new InvalidOperationException("Invalid constraint cache type.");
                }

                PairCache.Update<TConstraintCache, TCollisionCache>(workerIndex, index, ref pointers, ref collisionCache, &manifold->FeatureId0);
                if (manifoldTypeAsConstraintType == constraintCacheIndex.Type)
                {
                    //There exists a constraint and it has the same type as the manifold. Directly apply the new description and impulses.
                    Solver.ApplyDescription(ref constraintReference, ref description);
                    //TODO: Check init hack.
                    PairCache.ScatterNewImpulses(manifoldTypeAsConstraintType, ref constraintReference, ref *&newImpulses);
                }
                else
                {
                    //There exists a constraint, but it's a different type. This is more complex:
                    //1) The new manifold's constraint must be added, but upon the adder's return the solver is not guaranteed to contain the constraint- it may be deferred.
                    //(This allows a custom adder to implement deferred approaches. For example, determinism requires a consistent order of solver constraint addition, and a post-sort 
                    //can be used to guarantee that consistent order. We can also defer smaller batches for the sake of limiting sync overheads. 4-16 adds within a single lock
                    //means a 4-16x reduction in lock-related overhead, assuming no contests.)
                    //2) The old constraint must be removed.
                    RequestAddConstraint(workerIndex, constraintCacheIndex, &newImpulses, ref description, bodyHandles);
                    Callbacks.EnqueueConstraintRemoval(workerIndex, constraintHandle);
                }

            }
            else
            {
                //No preexisting constraint; add a fresh constraint and pair cache entry.
                //The pair cache entry has to be created first so that the adder has a place to put the result of the constraint add.
                var constraintCacheIndex = PairCache.Add<TConstraintCache, TCollisionCache>(workerIndex, ref pair, ref collisionCache, &manifold->FeatureId0);
                var newImpulses = new ContactImpulses();
                //TODO: It would be nice to avoid the impulse scatter for fully new constraints; it's going to be all zeroes regardless. Worth investigating later.
                RequestAddConstraint(workerIndex, constraintCacheIndex, &newImpulses, ref description, bodyHandles);
            }
        }

        unsafe void UpdateConstraintForManifold<TCollisionCache, TBodyHandles>(int workerIndex, ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache,
            ref PairMaterialProperties material, TBodyHandles bodyHandles)
            where TCollisionCache : IPairCacheEntry
        {
            //Note that this function has two responsibilities:
            //1) Create the description of the constraint that should represent the new manifold.
            //2) Add that constraint (or update an existing constraint) with that description, updating any accumulated impulses as needed.
            //Conceptually, it would be nicer to have a single function for each of these- create a description, and then apply it.
            //However, we cannot return the type knowledge we extract from the constraint cache index. Instead, we make use of the information in-place.

            //TODO: Should check codegen and alternatives here.
            //TODO: Descriptions will be changing to not have redundant B offsets, this will have to change to match.
            Debug.Assert(manifold->ContactCount > 0);
            //Constraint types only use 3 bits, since their contact count can never be zero.
            var manifoldTypeAsConstraintType = ((manifold->PackedConvexityAndContactCount >> 1) & 4) | ((manifold->PackedConvexityAndContactCount & 7) - 1);
            switch (manifoldTypeAsConstraintType)
            {
                //Convex
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
                    {
                        ContactManifold4Constraint description;
                        var descriptionContacts = &description.Contact0;
                        var offsets = &manifold->Offset0;
                        var depths = &manifold->Depth0;
                        for (int i = 0; i < 4; ++i)
                        {
                            ref var descriptionContact = ref descriptionContacts[i];
                            descriptionContact.OffsetA = offsets[i];
                            descriptionContact.OffsetB = offsets[i] - manifold->OffsetB;
                            descriptionContact.PenetrationDepth = depths[i];
                        }
                        description.FrictionCoefficient = material.FrictionCoefficient;
                        description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
                        description.SpringSettings = material.SpringSettings;
                        description.SurfaceBasis = manifold->ConvexSurfaceBasis;

                        //TODO: Check init hack.
                        UpdateConstraint<ContactManifold4Constraint, TCollisionCache, ConstraintCache4, TBodyHandles>(
                            workerIndex, ref pair, manifold, manifoldTypeAsConstraintType, ref collisionCache, ref *&description, bodyHandles);
                    }
                    break;
                //Nonconvex
                case 4 + 0:
                    break;
                case 4 + 1:
                    break;
                case 4 + 2:
                    break;
                case 4 + 3:
                    break;

            }

        }

        public unsafe void UpdateConstraintsForPair<TCollisionCache>(int workerIndex, ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache) where TCollisionCache : IPairCacheEntry
        {
            //Note that we do not check for the pair being between two statics before reporting it. The assumption is that, if the initial broadphase pair filter allowed such a pair
            //to reach this point, the user probably wants to receive some information about the resulting contact manifold.
            //That said, such a pair cannot generate constraints no matter what- constraints must involve at least one body, always.
            var aIsBody = !pair.A.IsStatic;
            var bIsBody = !pair.B.IsStatic;
            if (Callbacks.ConfigureContactManifold(workerIndex, pair, ref *manifold, out var pairMaterial) && (aIsBody || bIsBody))
            {
                if (manifold->ContactCount > 0)
                {
                    if (aIsBody && bIsBody)
                    {
                        //Two bodies.
                        var bodyHandles = new TwoBodyHandles { A = pair.A.Collidable, B = pair.B.Collidable };
                        UpdateConstraintForManifold(workerIndex, ref pair, manifold, ref collisionCache, ref pairMaterial, bodyHandles);
                    }
                    else
                    {
                        //One of the two collidables is static.
                        //Ensure that the body is always in the first slot for the sake of constraint generation.
                        if (bIsBody)
                        {
                            var tempA = pair.A;
                            pair.A = pair.B;
                            pair.B = tempA;
                        }
                        UpdateConstraintForManifold(workerIndex, ref pair, manifold, ref collisionCache, ref pairMaterial, pair.A.Collidable);
                    }
                }
                //In the event that there are no contacts in the new manifold, the pair is left in a stale state. It will be removed by the stale removal post process. 
            }
        }

        public unsafe void Execute(Buffer<PairJob> jobs, int jobStart, int jobCount, ref ConvexContactManifoldWide manifold)
        {
            //Given the use of virtuals elsewhere, it may seem a bit odd to use a hardcoded switch here.
            //Especially because these are not guaranteed to be the only types ever supported. Some extensions may demand new entries here.
            //As usual, this comes down to performance:
            //1) Continuation types vary on a per-pair basis, rather than on a per-batch basis. This was a tradeoff:
            //the greater the degree of batch segmentation, the less work could be batched together, and the more pending pairs sit around in L1 cache.
            //Since the contact manifold calculation is the dominant cost on nontrivial pairs, the more full-occupancy SIMD dispatches we can do, the better.

            //2) Having established per-pair invocation, virtual invocations tend to be slower than contiguous switch statements by about 8-15 cycles.
            //For 100000 pairs on a 3770K-like CPU, we'd expect a virtual implementation to run 50-100us slower than a switch implementation.
            //That's pretty small, but this kind of thing adds up. 

            //(This choice is something to monitor over time in terms of virtual/switch codegen, batching, and extensibility. We already do a virtual dispatch at the 
            //batch level, so if it turns out pairType x continuationType batching is okay in practice and the hardcodedness is getting in the way, we can switch.)
            var pairBase = (PairJob*)jobs.Memory + jobStart;
            for (int i = 0; i < jobCount; ++i)
            {
                ref var job = ref *(pairBase + i);
                switch (job.Continuation.Type)
                {
                    case ContinuationType.ConvexConstraintGenerator:

                        //if (PairCache.TryGetValue(ref job.Pair, out var constraintHandle))
                        //{
                        //    //This pair is associated with a constraint. 
                        //    GatherAccumulatedImpulses(constraintHandle, out var previousImpulses)




                        //}
                        //TODO: When we support non-constraint manifold storage targets for coldet-only use cases, we'll need to avoid attempting to make changes to the solver.
                        break;

                }
            }

        }
    }
}