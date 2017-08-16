using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System.Runtime.CompilerServices;
using SolverPrototype.Constraints;
using System.Diagnostics;
using System.Numerics;
using SolverPrototype.Collidables;

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


namespace SolverPrototype.CollisionDetection
{ 
    public struct Continuations<TFilters, TConstraintAdder, TConstraintRemover, TCollidableData>
        where TFilters : INarrowPhaseFilters where TConstraintAdder : INarrowPhaseConstraintAdder where TConstraintRemover : INarrowPhaseConstraintRemover where TCollidableData : struct
    {
        public Solver Solver;
        public NarrowPhase<TFilters, TConstraintAdder, TConstraintRemover, TCollidableData> NarrowPhase;
        public Continuations(Solver solver, NarrowPhase<TFilters, TConstraintAdder, TConstraintRemover, TCollidableData> narrowPhase)
        {
            Solver = solver;
            NarrowPhase = narrowPhase;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void AddConstraint4(int workerIndex, TypedIndex constraintCacheIndex, int bodyHandleA, int bodyHandleB, ref ConvexContactManifold manifold, ref PairMaterialProperties pairMaterial)
        {
            ContactManifold4Constraint description;
            var constraintContactData = &description.Contact0;
            for (int i = 0; i < 4; ++i)
            {
                ref var target = ref constraintContactData[i];
                ref var source = ref manifold[i];
                target.OffsetA = source.Offset;
                target.OffsetB = source.Offset - manifold.OffsetB;
                target.PenetrationDepth = source.Depth;
            }
            description.SurfaceBasis = manifold.SurfaceBasis;
            description.MaximumRecoveryVelocity = pairMaterial.MaximumRecoveryVelocity;
            description.SpringSettings = pairMaterial.SpringSettings;
            description.FrictionCoefficient = pairMaterial.FrictionCoefficient;

            //TODO: Check codegen on that little anti-init hack. If it's worse than directly passing a pointer, directly pass a pointer.
            NarrowPhase.ConstraintAdder.AddConstraint(workerIndex, constraintCacheIndex, bodyHandleA, bodyHandleB, ref *&description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void RedistributeImpulses(int oldContactCount, float* oldImpulses, int* oldFeatureIds, ref ConvexContactManifold manifold, float* newImpulses)
        {
            //Map the new contacts to the old contacts. 
            for (int i = 0; i < manifold.ContactCount; ++i)
            {
                newImpulses[i] = 0;
                for (int j = 0; j < oldContactCount; ++j)
                {
                    if (oldFeatureIds[j] == manifold[i].FeatureId)
                    {
                        newImpulses[i] = oldImpulses[j];
                    }
                }
            }
            //TODO: 'Unclaimed' impulse from old unmatched contacts could be redistributed to try to conserve total impulse. Something to fiddle with once we have a test case running.
        }

        public unsafe void UpdateConstraintsForPair(CollidablePair pair, ref ConvexContactManifold manifold, int workerIndex)
        {
            //TODO: To support efficient static-body constraints, we'll need to create permutations of this function.
            Debug.Assert(!pair.A.IsStatic && !pair.B.IsStatic, "When using the two body constraint update, both involved collidables should belong to bodies!");
            if (NarrowPhase.Filters.ConfigureContactManifold(workerIndex, pair, ref manifold, out var pairMaterial))
            {
                switch (manifold.ContactCount)
                {
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        {
                            //Is this constraint new?
                            if (NarrowPhase.PairCache.TryGetPointers(ref pair, out var pointers))
                            {
                                //TODO: Probably worth using an explicit non-TypedIndex type for this encoding.
                                //8 types, 0-3 convex, 4-7 nonconvex.
                                var isNonconvex = pointers.ConstraintCache.Type & 4;
                                ref var constraintCache = ref NarrowPhase.PairCache.GetConstraintCache<ConstraintCache4>(pointers.ConstraintCache.Index);
                                const int newContactCount = 4;
                                if (cachedPairData.Convex && cachedPairData.ContactCount == newContactCount)
                                {
                                    //This is the same constraint type. We can modify the old constraint without worrying about adding or removing constraints.
                                    Solver.GetConstraintReference(cachedPairData.Handle, out var reference);
                                    var batch = Unsafe.As<TypeBatch, ContactManifold4TypeBatch>(ref reference.TypeBatch);
                                    BundleIndexing.GetBundleIndices(reference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
                                    //Note that convex-convex transitions don't really require any central friction accumulated impulse changes. They won't tend to change significantly.
                                    ref var bundle = ref batch.AccumulatedImpulses[bundleIndex];
                                    var oldImpulse0 = (float*)Unsafe.AsPointer(ref bundle.Penetration0) + innerIndex;
                                    var oldImpulse1 = oldImpulse0 + Vector<float>.Count;
                                    var oldImpulse2 = oldImpulse0 + Vector<float>.Count * 2;
                                    var oldImpulse3 = oldImpulse0 + Vector<float>.Count * 3;
                                    var oldImpulses = stackalloc float[newContactCount];
                                    var newImpulses = stackalloc float[newContactCount];
                                    oldImpulses[0] = *oldImpulse0;
                                    oldImpulses[1] = *oldImpulse1;
                                    oldImpulses[2] = *oldImpulse2;
                                    oldImpulses[3] = *oldImpulse3;
                                    //Could technically specialize for single type to avoid some of the allocating, but only do that if it shows up in profiling.
                                    RedistributeImpulses(newContactCount, oldImpulses, &cachedPairData.FeatureId0, ref manifold, newImpulses);
                                    *oldImpulse0 = newImpulses[0];
                                    *oldImpulse1 = newImpulses[1];
                                    *oldImpulse2 = newImpulses[2];
                                    *oldImpulse3 = newImpulses[3];
                                }
                                else
                                {
                                    //This is a different constraint type. Either the convexity changed or the contact count changed; either way, a new constraint must be created.
                                    //This requires two parts:
                                    //1) Add the new constraint. We defer to the callback for the implementation;
                                    //2) Add the old constraint to a 'to remove' set. This is not just for the sake of synchronization efficiency. We can only trigger removals after all 
                                    //constraint modifications are complete. Removals can change the order of constraints in a type batch, so if removals were allowed during this phase,
                                    //even the case where the previous and current type are the same would require synchronization. That's really bad, since state transitions are actually relatively rare.
                                    //The good news is that we can handle constraint removals in parallel in the same way that we do for deactivation, so even if there is a ton of constraint churn,
                                    //multithread utilization will still be high.
                                    AddConstraint4(pair.A.Collidable, pair.B.Collidable, ref manifold, ref pairMaterial, impulses);

                                    //Any existing constraint should be 

                                    NarrowPhase.ConstraintRemover.EnqueueConstraintRemoval(cachedPairData.Handle);

                                }
                            }
                            else
                            {
                                //This is a new constraint without precedent.
                                AddConstraint4(pair.A.Collidable, pair.B.Collidable, ref manifold, ref pairMaterial);
                            }
                        }
                        break;
                }
            }

            ////TODO: If there is any measurable value, you could technically create another two versions of this function that use type knowledge to guarantee
            ////whether the previous manifold is convex or not.
            ////Also, you could use type knowledge to guarantee a maximum number of contacts. Sphere-sphere is always going to be 1, after all.
            //if (NarrowPhase.PairCache.TryGetValue(pair, out var cachedPairData))
            //{
            //    Solver.GetConstraintReference(cachedPairData.Handle, out var reference);
            //    if (cachedPairData.Convex)
            //    {
            //        var oldContactCount = cachedPairData.ContactCount;
            //        var oldAccumulatedImpulses = stackalloc float[oldContactCount];
            //        switch (cachedPairData.ContactCount)
            //        {
            //            case 1:
            //                break;
            //            case 2:
            //                break;
            //            case 3:
            //                break;
            //            case 4:
            //                var batch = Unsafe.As<TypeBatch, ContactManifold4TypeBatch>(ref reference.TypeBatch);
            //                ref var accumulatedImpulsesBundle = ref batch.AccumulatedImpulses[reference.IndexInTypeBatch];
            //                BundleIndexing.GetBundleIndices(reference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);

            //                //TODO: Check codegen. Backing memory is pinned, so we could take pointers to improve it if necessary.
            //                oldAccumulatedImpulses[0] = accumulatedImpulsesBundle.Penetration0[innerIndex];
            //                oldAccumulatedImpulses[1] = accumulatedImpulsesBundle.Penetration1[innerIndex];
            //                oldAccumulatedImpulses[2] = accumulatedImpulsesBundle.Penetration2[innerIndex];
            //                oldAccumulatedImpulses[3] = accumulatedImpulsesBundle.Penetration3[innerIndex];
            //                break;
            //        }

            //        //Map the new contacts to the old contacts. 
            //        var sourceIndices = stackalloc[manifold.Count];
            //        for (int i = 0; i < manifold.Count; ++i)
            //        {
            //            sourceIndices[i] = -1;
            //            for (int j = 0; j < oldContactCount; ++j)
            //            {
            //                if (CachedPairData.GetFeatureId(ref cachedPairData, j) == ConvexContactManifold.GetFeatureId(ref manifold, i)) ;
            //                {
            //                    sourceIndices[i] = j;
            //                }
            //            }
            //        }

            //        //For convex manifolds, we leave the central friction untouched. The positions of contacts in the manifold may actually have changed the best guess, but
            //        //twist friction is rarely such a critical factor in accumulated impulses that a 10% change in manifold radius has any perceivable impact.





            //        for (int i = 0; i < manifold.Count; ++i)
            //        {
            //            oldAccumulatedImpulses[sourceIndices[i]];
            //        }

            //        //TODO: You may want to try redistributing any 'unclaimed' accumulated impulse over contacts that had no match. This could cause overshoot, but 
            //        //it might also improve stacking in some cases. Simple to add- just try it out once you have a the ability to test stability meaningfully.
            //    }
            //    else
            //    {
            //        if (manifold.Convex)
            //        {

            //        }
            //        else
            //        {

            //        }
            //    }
            //}

            ////This establishes a hardcoded relationship between narrow phase pairs and solver types. This could cause some awkwardness, but it is a relatively simple approach
            ////that avoids the need for virtual invocation when gathering and scattering accumulated impulses.
            ////TODO: If greater extensibility is required later, you could consider storing data which allow the direct calculation of accumulated impulse pointers without knowing the type.
            ////There's a little complexity there in that we still have to be mindful of the convex vs nonconvex constraint split, but it could be made to work. The data would likely already
            ////get pulled into cache because the type batch's accumulated impulses and prestep data pointers have to be pulled in.

            ////This callback is responsible for both determining if the constraint should exist at all and for filling any necessary material properties.
            ////It also acts as user notification of the manifold. An event system could be built on top of this.
            //if (NarrowPhase.Callbacks.AllowConstraint(collidableReferenceA, collidableReferenceB, ref constraintDescription))
            //{
            //    //So now we assume that the material properties are set and this constraint is ready to be put into the solver itself.
            //    OldManifold oldManifold;
            //    var oldAccumulatedImpulses = &oldManifold.Penetration0;
            //    if (targetType == constraint.Type)
            //    {
            //        //We can directly change the accumulated impulses within the constraint.
            //        //No new constraint needs to be created.
            //        Solver.GetConstraintReference(constraint.Handle, out var reference);
            //        switch (constraint.Type)
            //        {
            //            case PairConstraintType.Convex1:
            //                break;
            //            case PairConstraintType.Convex2:
            //                break;
            //            case PairConstraintType.Convex3:
            //                break;
            //            case PairConstraintType.Convex4:
            //                {
            //                    oldManifold.Count = 4;


            //                }
            //                break;
            //            case PairConstraintType.Nonconvex1:
            //                break;
            //            case PairConstraintType.Nonconvex2:
            //                break;
            //            case PairConstraintType.Nonconvex3:
            //                break;
            //            case PairConstraintType.Nonconvex4:
            //                break;
            //        }
            //    }
            //    else
            //    {
            //        //The constraint type is changing, so we need to prepare a removal for the old constraint while setting up a new constraint.
            //        //This requires two parts:
            //        //1) Add the new constraint. For now, we use a simple spinlock to directly add the new constraint. Adding constraints to the solver is guaranteed to not change
            //        //any existing constraint index, so it's fine to do it at the same time as other threads are still working on updating accumulated impulses.
            //        //Later on, we may want to look into batching adds. Even a spinlock is far from free, especially if there is any contest. If we could batch up a number of adds at a time,
            //        //we may be able to reduce synchronization overhead and get better throughput. On the other hand, such a batching process would require that we store the pending 
            //        //constraints, which will add cache pressure and lead to more stalls, and it could be that the spinlock is sufficiently rarely contested that it is a trivial cost.
            //        //Experimentation will be required.
            //        //2) Add the old constraint to a 'to remove' set. This is not just for the sake of synchronization efficiency. We can only trigger removals after all 
            //        //constraint modifications are complete. Removals can change the order of constraints in a type batch, so if removals were allowed during this phase,
            //        //even the case where the previous and current type are the same would require synchronization. That's really bad, since state transitions are actually relatively rare.
            //        //The good news is that we can handle constraint removals in parallel in the same way that we do for deactivation, so even if there is a ton of constraint churn,
            //        //multithread utilization will still be high.

            //        bool taken = false;
            //        NarrowPhase.AddLock.TryEnter(ref taken);
            //        switch (targetType)
            //        {
            //            case PairConstraintType.Convex1:
            //                break;
            //            case PairConstraintType.Convex2:
            //                break;
            //            case PairConstraintType.Convex3:
            //                break;
            //            case PairConstraintType.Convex4:
            //                {
            //                    ContactManifold4Constraint description;
            //                    description.
            //                }
            //                break;
            //            case PairConstraintType.Nonconvex1:
            //                break;
            //            case PairConstraintType.Nonconvex2:
            //                break;
            //            case PairConstraintType.Nonconvex3:
            //                break;
            //            case PairConstraintType.Nonconvex4:
            //                break;
            //        }
            //        NarrowPhase.AddLock.Exit();
            //    }
            //}



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