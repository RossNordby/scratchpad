﻿using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.Constraints
{
    public abstract class TypeBatch
    {
        //TODO: Having this in the base class actually complicates the implementation of some special constraint types. Consider an 'articulation' subsolver that involves
        //N bodies, for N > Vector<float>.Count * 2. You may want to do SIMD internally in such a case, so there would be no 'bundles' at this level. Worry about that later.
        protected int bundleCount;
        public int BundleCount => bundleCount;
        protected int constraintCount;
        protected int typeId;
        public int ConstraintCount => constraintCount;

        public abstract int BodiesPerConstraint { get; }

        /// <summary>
        /// The handles for the constraints in this type batch.
        /// </summary>
        public int[] IndexToHandle;

        /// <summary>
        /// Allocates a slot in the batch.
        /// </summary>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <param name="bodyIndices">Pointer to a list of body indices (not handles!) with count equal to the type batch's expected number of involved bodies.</param>
        /// <returns>Index of the slot in the batch.</returns>
        public unsafe abstract int Allocate(int handle, int* bodyIndices);
        public abstract void Remove(int index, ConstraintLocation[] handlesToConstraints);
        public abstract void TransferConstraint(int indexInTypeBatch, Solver solver, int targetBatch);

        public abstract void EnumerateConnectedBodyIndices<TEnumerator>(int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int>;
        public abstract void UpdateForBodyMemoryMove(int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation);

        public abstract void Scramble(Random random, ConstraintLocation[] handlesToConstraints);

        /// <summary>
        /// Sorts a subset of constraints in the type batch according to the location of bodies in memory. The goal is to maximize cache coherence.
        /// </summary>
        /// <param name="bundleStartIndex">Start of the sorting region.</param>
        /// <param name="constraintCount">Number of constraints (not bundles!) to sort.</param>
        /// <param name="handlesToConstraints">The handle to constraint mapping used by the solver that needs to be updated in response to swaps.</param>
        /// <param name="bodyCount">Number of bodies in the body set.</param>
        public abstract void SortByBodyLocation(int bundleStartIndex, int constraintCount, ConstraintLocation[] handlesToConstraints, int bodyCount);

        public abstract void Initialize(int initialCapacityInBundles, int typeId);
        public abstract void Reset();

        public abstract void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt, int startBundle, int endBundle);
        public abstract void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle);
        public abstract void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle);



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt)
        {
            Prestep(bodyInertias, dt, inverseDt, 0, bundleCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(BodyVelocities[] bodyVelocities)
        {
            WarmStart(bodyVelocities, 0, bundleCount);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SolveIteration(BodyVelocities[] bodyVelocities)
        {
            SolveIteration(bodyVelocities, 0, bundleCount);
        }

        [Conditional("DEBUG")]
        public abstract void ValidateBundleCounts();

    }
    //You are allowed to squint at this triple-class separation.
    //This is only really here because there are cases (e.g. adding a constraint) where it is necessary to have knowledge of TBodyReferences so that the caller (the solver, generally)
    //can communicate to the type batch in a type safe way. The alternative would have been including the TPrestepData, TProjection, and TAccumulatedImpulse, which just gets excessive.
    //Avoiding generic type knowledge would likely have involved some goofy safe-but-Unsafe casting.
    //We might have some issues with this in the future if we have fully unconstrained body reference counts in a constraint. It's wise to avoid that.

    public abstract class TypeBatch<TBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse> : TypeBatch
    {
        public TBodyReferences[] BodyReferences;
        public TPrestepData[] PrestepData;
        //Technically, the iteration data does not need to persist outside of the scope of the solve. We let it persist for simplicity- it does not take much space.

        //Projection and unprojection data required by the WarmStart and SolveIteration. Note that this data is conceptually ephemeral.
        //External users should not depend upon it outside of the solver's execution.
        //(At the moment, it does persist, but it becomes unreliable when constraints are removed, and the implementation reserves the right to make it completely temporary.)
        protected TProjection[] Projection;
        public TAccumulatedImpulse[] AccumulatedImpulses;


        static void IncreaseSize<T>(ref T[] array)
        {
            //Out of space. Need to resize.
            var old = array;
            array = BufferPools<T>.Locking.Take(array.Length << 1);
            Array.Copy(old, array, old.Length);
            BufferPools<T>.Locking.Return(old);
        }

        protected unsafe abstract void AddBodyReferences(int index, int* bodyIndices);
        protected abstract void RemoveBodyReferences(int bundleIndex, int innerIndex);

        /// <summary>
        /// Allocates a slot in the batch without filling the body references.
        /// </summary>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        internal int Allocate(int handle)
        {
            Debug.Assert(Projection != null, "Should initialize the batch before allocating anything from it.");
            if (constraintCount == IndexToHandle.Length)
            {
                IncreaseSize(ref BodyReferences);
                IncreaseSize(ref PrestepData);
                IncreaseSize(ref Projection);
                IncreaseSize(ref AccumulatedImpulses);
                IncreaseSize(ref IndexToHandle);
            }
            var index = constraintCount++;
            IndexToHandle[index] = handle;
            if ((constraintCount & BundleIndexing.VectorMask) == 1)
                ++bundleCount;
            return index;
        }

        /// <summary>
        /// Allocates a slot in the batch.
        /// </summary>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <param name="bodyIndices">Pointer to a list of body indices (not handles!) with count equal to the type batch's expected number of involved bodies.</param>
        /// <returns>Index of the slot in the batch.</returns>
        public unsafe sealed override int Allocate(int handle, int* bodyIndices)
        {
            var index = Allocate(handle);
            AddBodyReferences(index, bodyIndices);
            return index;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void CopyConstraintData(
             ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle, int sourceInner,
             ref TBodyReferences targetReferencesBundle, ref TPrestepData targetPrestepBundle, ref TAccumulatedImpulse targetAccumulatedBundle, int targetInner)
        {
            //Note that we do NOT copy the iteration data. It is regenerated each frame from scratch. 
            //We may later decide that this is silly because someone might rely on it, but... it seems very unlikely. 
            //Try to stop people from relying on it, and see if anyone ever complains.
            GatherScatter.CopyLane(ref sourceReferencesBundle, sourceInner, ref targetReferencesBundle, targetInner);
            GatherScatter.CopyLane(ref sourcePrestepBundle, sourceInner, ref targetPrestepBundle, targetInner);
            GatherScatter.CopyLane(ref sourceAccumulatedBundle, sourceInner, ref targetAccumulatedBundle, targetInner);
        }

        /// <summary>
        /// Overwrites all the data in the target constraint slot with source data.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(
            ref TBodyReferences sourceReferencesBundle, ref TPrestepData sourcePrestepBundle, ref TAccumulatedImpulse sourceAccumulatedBundle,
            int sourceInner, int sourceHandle, int targetBundle, int targetInner, int targetIndex, ConstraintLocation[] handlesToConstraints)
        {
            CopyConstraintData(
                ref sourceReferencesBundle, ref sourcePrestepBundle, ref sourceAccumulatedBundle, sourceInner,
                ref BodyReferences[targetBundle], ref PrestepData[targetBundle], ref AccumulatedImpulses[targetBundle], targetInner);
            IndexToHandle[targetIndex] = sourceHandle;
            handlesToConstraints[sourceHandle].IndexInTypeBatch = targetIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(int sourceBundle, int sourceInner, int sourceIndex, int targetBundle, int targetInner, int targetIndex,
             ConstraintLocation[] handlesToConstraints)
        {
            Move(ref BodyReferences[sourceBundle], ref PrestepData[sourceBundle], ref AccumulatedImpulses[sourceBundle], sourceInner, IndexToHandle[sourceIndex],
                targetBundle, targetInner, targetIndex, handlesToConstraints);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(int sourceIndex, int targetIndex,
            ConstraintLocation[] handlesToConstraints)
        {
            BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
            BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);
            Move(ref BodyReferences[sourceBundle], ref PrestepData[sourceBundle], ref AccumulatedImpulses[sourceBundle], sourceInner, IndexToHandle[sourceIndex],
                targetBundle, targetInner, targetIndex, handlesToConstraints);
        }


        public sealed override void Scramble(Random random, ConstraintLocation[] handlesToConstraints)
        {
            //This is a pure debug function used to compare cache optimization strategies. Performance doesn't matter. 
            TPrestepData aPrestep = default(TPrestepData);
            TAccumulatedImpulse aAccumulated = default(TAccumulatedImpulse);
            TBodyReferences aBodyReferences = default(TBodyReferences);
            int aHandle;

            for (int a = ConstraintCount - 1; a >= 1; --a)
            {
                BundleIndexing.GetBundleIndices(a, out var aBundle, out var aInner);
                GatherScatter.CopyLane(ref BodyReferences[aBundle], aInner, ref aBodyReferences, 0);
                GatherScatter.CopyLane(ref PrestepData[aBundle], aInner, ref aPrestep, 0);
                GatherScatter.CopyLane(ref AccumulatedImpulses[aBundle], aInner, ref aAccumulated, 0);
                aHandle = IndexToHandle[a];

                var b = random.Next(a);
                BundleIndexing.GetBundleIndices(b, out var bBundle, out var bInner);
                Move(bBundle, bInner, b, aBundle, aInner, a, handlesToConstraints);
                Move(ref aBodyReferences, ref aPrestep, ref aAccumulated, 0, aHandle, bBundle, bInner, b, handlesToConstraints);
            }
        }

        /// <summary>
        /// Removes a constraint from the batch.
        /// </summary>
        /// <param name="index">Index of the constraint to remove.</param>
        /// <param name="handlesToConstraints">The handle to constraint mapping used by the solver that could be modified by a swap on removal.</param>
        public override void Remove(int index, ConstraintLocation[] handlesToConstraints)
        {
            Debug.Assert(index >= 0 && index < constraintCount, "Can only remove elements that are actually in the batch!");
            var lastIndex = constraintCount - 1;
            constraintCount = lastIndex;
            if ((constraintCount & BundleIndexing.VectorMask) == 0)
                --bundleCount;
            BundleIndexing.GetBundleIndices(lastIndex, out var sourceBundleIndex, out var sourceInnerIndex);
            if (index < lastIndex)
            {
                //Need to swap.
                BundleIndexing.GetBundleIndices(index, out var targetBundleIndex, out var targetInnerIndex);
                Move(sourceBundleIndex, sourceInnerIndex, lastIndex, targetBundleIndex, targetInnerIndex, index, handlesToConstraints);
            }
            //Clear the last slot's accumulated impulse regardless of whether a swap takes place. This avoids new constraints getting a weird initial guess.
            GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref AccumulatedImpulses[sourceBundleIndex], sourceInnerIndex);
            RemoveBodyReferences(sourceBundleIndex, sourceInnerIndex);
        }

        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="targetBatch">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        public override void TransferConstraint(int indexInTypeBatch, Solver solver, int targetBatch)
        {
            //This cast is pretty gross looking, but guaranteed to work by virtue of the typeid registrations.
            var targetTypeBatch = (TypeBatch<TBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>)
                solver.Batches.Elements[targetBatch].GetOrCreateTypeBatch(typeId, solver.TypeBatchAllocation);
            var handle = IndexToHandle[indexInTypeBatch];
            //Allocate a target slot. Don't put references into the target slot via the usual means- we'll just copy directly in.
            var indexInTargetTypeBatch = targetTypeBatch.Allocate(handle);
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var sourceBundle, out var sourceInner);
            BundleIndexing.GetBundleIndices(indexInTargetTypeBatch, out var targetBundle, out var targetInner);
            //Note that we leave out the runtime generated bits- they'll just get regenerated.
            GatherScatter.CopyLane(ref BodyReferences[sourceBundle], sourceInner, ref targetTypeBatch.BodyReferences[targetBundle], targetInner);
            GatherScatter.CopyLane(ref PrestepData[sourceBundle], sourceInner, ref targetTypeBatch.PrestepData[targetBundle], targetInner);
            GatherScatter.CopyLane(ref AccumulatedImpulses[sourceBundle], sourceInner, ref targetTypeBatch.AccumulatedImpulses[targetBundle], targetInner);
            //Get rid of the constraint in the current batch.
            Remove(indexInTypeBatch, solver.HandlesToConstraints);
            //Don't forget to keep the solver's pointers consistent! We bypassed the usual add procedure, so the solver hasn't been notified yet.
            ref var constraintLocation = ref solver.HandlesToConstraints[handle];
            constraintLocation.BatchIndex = targetBatch;
            constraintLocation.IndexInTypeBatch = indexInTargetTypeBatch;
            //Typeid is unchanged.
            Debug.Assert(constraintLocation.TypeId == typeId && targetTypeBatch.typeId == typeId);

            //Note that this implementation is a little bit dangerous in the sense that it creates more codepaths that must maintain synchronization with all the others.
            //If we added more bookkeeping bits in one, the others might end up broken. But, in this case, it's pretty useful- the overhead required for a full remove-add
            //is very high compared to a typebatch-typebatch copy.
            //Basically, annoyance for performance, like many things in BEPUphysics v2. This is a sequential operation, so reducing overhead is important.
            //You'll see this tradeoff repeated in the deactivation system where it preprocesses adds into bulk copy operations, 
            //somewhat similar to the above (except for many constraints and bodies at once).
        }

        public override void Initialize(int initialCapacityInBundles, int typeId)
        {
            Projection = BufferPools<TProjection>.Locking.Take(initialCapacityInBundles);
            BodyReferences = BufferPools<TBodyReferences>.Locking.Take(initialCapacityInBundles);
            PrestepData = BufferPools<TPrestepData>.Locking.Take(initialCapacityInBundles);
            AccumulatedImpulses = BufferPools<TAccumulatedImpulse>.Locking.Take(initialCapacityInBundles);
            IndexToHandle = BufferPools<int>.Locking.Take(initialCapacityInBundles * Vector<float>.Count);
            //Including this in the typebatch is a little bit hacky; it's just useful to know the typeid without any explicit generic parameters within the type batch 
            //when doing things like the MoveConstraint above- being able to know which type batch without external help simplifies the usage.
            //Given that the initializer always knows the typeid, and given that the cost is an extra 4 bytes at the level of type batches, this hack is pretty low concern.
            //Technically, this could be looked up in the ConstraintTypeIds static class by using this.GetType(), but that's a slow path.
            this.typeId = typeId;
        }

        public override void Reset()
        {
            BufferPools<TProjection>.Locking.Return(Projection);
            BufferPools<TBodyReferences>.Locking.Return(BodyReferences);
            BufferPools<TPrestepData>.Locking.Return(PrestepData);
            BufferPools<TAccumulatedImpulse>.Locking.Return(AccumulatedImpulses);
            BufferPools<int>.Locking.Return(IndexToHandle);
            Projection = null;
            BodyReferences = null;
            AccumulatedImpulses = null;
            AccumulatedImpulses = null;
            IndexToHandle = null;
        }



    }
}
