using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System.Runtime.CompilerServices;
using SolverPrototype.Constraints;
using System.Diagnostics;
using System.Numerics;
using SolverPrototype.Collidables;
using System;

namespace SolverPrototype.CollisionDetection
{
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

    public struct ContactImpulses1
    {
        public float Impulse0;
        public float Impulse1;
    }
    public struct ContactImpulses2
    {
        public float Impulse0;
        public float Impulse1;
    }
    public struct ContactImpulses3
    {
        public float Impulse0;
        public float Impulse1;
        public float Impulse2;
    }
    public struct ContactImpulses4
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

            //Kinda goofy workaround for lack of blittable generics. Just avoiding a zero init.
            //(Sort of. In order for the stackalloc to not zero init, you have to use the ReleaseStrip build config that includes the ilopt build step.
            //Otherwise, you'll have to wait for some corefx or coreclr change that allows you to suppress the localsinit flag.)
            var constraintCacheBytes = stackalloc byte[Unsafe.SizeOf<TConstraintCache>()];
            ref var newConstraintCache = ref Unsafe.As<byte, TConstraintCache>(ref *constraintCacheBytes);
            PairCache.FillNewConstraintCache(&manifold->FeatureId0, ref newConstraintCache);

            var index = PairCache.IndexOf(ref pair);
            if (index >= 0)
            {
                //The previous frame had a constraint for this pair.
                ref var pointers = ref PairCache.GetPointers(index);
                Debug.Assert(pointers.ConstraintCache.Exists, "If a pair was persisted in the narrow phase, there should be a constraint associated with it.");

                var constraintCacheIndex = pointers.ConstraintCache;
                var constraintCachePointer = PairCache.GetOldConstraintCachePointer(index);
                var constraintHandle = *(int*)constraintCachePointer;
                ContactImpulses newImpulses;
                Solver.GetConstraintReference(constraintHandle, out var constraintReference);
                //TODO: Check codegen; this if statement should JIT to a single path.
                //We specialize the 1-contact case since the 'redistribution' is automatic.
                if (typeof(TConstraintCache) == typeof(ConstraintCache1))
                {
                    PairCache.GatherOldImpulses(constraintCacheIndex.Type, ref constraintReference, &newImpulses.Impulse0, out var oldContactCount);
                }
                else
                {
                    ContactImpulses oldImpulses;
                    PairCache.GatherOldImpulses(constraintCacheIndex.Type, ref constraintReference, &oldImpulses.Impulse0, out var oldContactCount);
                    //The first slot in the constraint cache is the constraint handle; the following slots are feature ids.
                    RedistributeImpulses(oldContactCount, &oldImpulses.Impulse0, (int*)constraintCachePointer + 1, manifold, &newImpulses.Impulse0);
                }

                if (manifoldTypeAsConstraintType == constraintCacheIndex.Type)
                {
                    //Since the old constraint is the same type, we aren't going to remove the old constraint and add a new one. That means no deferred process is going
                    //to update the constraint cache's constraint handle. The good news is that we already have a valid constraint handle from the pre-existing constraint.
                    //It's exactly the same type, so we can just overwrite its properties without worry.
                    //Note that we rely on the constraint handle being stored in the first 4 bytes of the constraint cache.
                    *(int*)constraintCacheBytes = constraintHandle;
                    PairCache.Update(workerIndex, index, ref pointers, ref collisionCache, ref newConstraintCache, manifoldTypeAsConstraintType);
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
                    PairCache.Update(workerIndex, index, ref pointers, ref collisionCache, ref newConstraintCache, manifoldTypeAsConstraintType);
                    RequestAddConstraint(workerIndex, constraintCacheIndex, &newImpulses, ref description, bodyHandles);
                    ConstraintRemover.EnqueueRemoval(workerIndex, constraintHandle);
                }                
            }
            else
            {
                //No preexisting constraint; add a fresh constraint and pair cache entry.
                //The pair cache entry has to be created first so that the adder has a place to put the result of the constraint add.
                var constraintCacheIndex = PairCache.Add(workerIndex, ref pair, ref collisionCache, ref newConstraintCache, manifoldTypeAsConstraintType);
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
            //1-4 contacts: 0x3
            //nonconvex: 0x4
            //1 body versus 2 body: 0x8
            //TODO: Very likely that we'll expand the nonconvex manifold maximum to 8 contacts, so this will need to be adjusted later.
            var manifoldTypeAsConstraintType = ((manifold->PackedConvexityAndContactCount >> 1) & 4) | ((manifold->PackedConvexityAndContactCount & 7) - 1);
            if (typeof(TBodyHandles) == typeof(TwoBodyHandles))
                manifoldTypeAsConstraintType |= 0x8;
            switch (manifoldTypeAsConstraintType)
            {
                //One body
                //Convex
                case 0:
                    break;
                case 1:
                    break;
                case 2:
                    break;
                case 3:
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
                //Two body
                //Convex
                case 8 + 0:
                    {
                        ContactManifold1Constraint description;
                        description.Contact0.OffsetA = manifold->Offset0;
                        description.Contact0.PenetrationDepth = manifold->Depth0;
                        description.OffsetB = manifold->OffsetB;
                        description.FrictionCoefficient = material.FrictionCoefficient;
                        description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
                        description.SpringSettings = material.SpringSettings;
                        description.Normal = manifold->ConvexNormal;
                        
                        //TODO: Check init hack.
                        UpdateConstraint<ContactManifold1Constraint, TCollisionCache, ConstraintCache1, TBodyHandles>(
                            workerIndex, ref pair, manifold, manifoldTypeAsConstraintType, ref collisionCache, ref *&description, bodyHandles);
                    }
                    break;
                case 8 + 1:
                    break;
                case 8 + 2:
                    break;
                case 8 + 3:
                    {
                        ContactManifold4Constraint description;
                        var descriptionContacts = &description.Contact0;
                        var offsets = &manifold->Offset0;
                        var depths = &manifold->Depth0;
                        for (int i = 0; i < 4; ++i)
                        {
                            ref var descriptionContact = ref descriptionContacts[i];
                            descriptionContact.OffsetA = offsets[i];
                            descriptionContact.PenetrationDepth = depths[i];
                        }
                        description.OffsetB = manifold->OffsetB;
                        description.FrictionCoefficient = material.FrictionCoefficient;
                        description.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
                        description.SpringSettings = material.SpringSettings;
                        description.Normal = manifold->ConvexNormal;

                        //TODO: Check init hack.
                        UpdateConstraint<ContactManifold4Constraint, TCollisionCache, ConstraintCache4, TBodyHandles>(
                            workerIndex, ref pair, manifold, manifoldTypeAsConstraintType, ref collisionCache, ref *&description, bodyHandles);
                    }
                    break;
                //Nonconvex
                case 8 + 4 + 0:
                    break;
                case 8 + 4 + 1:
                    break;
                case 8 + 4 + 2:
                    break;
                case 8 + 4 + 3:
                    break;

            }

        }

        public unsafe void UpdateConstraintsForPair<TCollisionCache>(int workerIndex, ref CollidablePair pair, ContactManifold* manifold, ref TCollisionCache collisionCache) where TCollisionCache : IPairCacheEntry
        {
            //Note that we do not check for the pair being between two statics before reporting it. The assumption is that, if the initial broadphase pair filter allowed such a pair
            //to reach this point, the user probably wants to receive some information about the resulting contact manifold.
            //That said, such a pair cannot generate constraints no matter what- constraints must involve at least one body, always.
            var aMobility = pair.A.Mobility;
            var bMobility = pair.B.Mobility;
            if (Callbacks.ConfigureContactManifold(workerIndex, pair, manifold, out var pairMaterial) &&
                //Note that, even if the callback says 'yeah sure create a constraint for those', it never makes sense to generate constraints between two nondynamics.
                //It would just result in a bunch of NaNs when computing the effective mass.
                (aMobility == CollidableMobility.Dynamic || bMobility == CollidableMobility.Dynamic))
            {
                if (manifold->ContactCount > 0)
                {
                    var aIsBody = aMobility != CollidableMobility.Static;
                    var bIsBody = bMobility != CollidableMobility.Static;
                    if (aIsBody && bIsBody)
                    {
                        //Two bodies.
                        var bodyHandles = new TwoBodyHandles { A = pair.A.Handle, B = pair.B.Handle };
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
                        UpdateConstraintForManifold(workerIndex, ref pair, manifold, ref collisionCache, ref pairMaterial, pair.A.Handle);
                    }
                }
                //In the event that there are no contacts in the new manifold, the pair is left in a stale state. It will be removed by the stale removal post process. 
            }
        }

    }
}