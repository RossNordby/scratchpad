using BEPUutilities2.Collections;
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

        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceBatch">Batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        public abstract void TransferConstraint(ConstraintBatch sourceBatch, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex);

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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void AddBodyReferencesLane(ref TBodyReferences bundle, int innerIndex, int* bodyIndices)
        {
            //The jit should be able to fold almost all of the size-related calculations and address fiddling.
            ref var start = ref Unsafe.As<TBodyReferences, int>(ref bundle);
            ref var targetLane = ref Unsafe.Add(ref start, innerIndex);
            var stride = Vector<int>.Count * 2;
            //We assume that the body references struct is organized in memory like Bundle0, Inner0, ... BundleN, InnerN, Count
            //Assuming contiguous storage, Count is then located at start + stride * BodyCount.
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / (stride * 4);
            Debug.Assert(innerIndex == 0 || Unsafe.Add(ref start, stride * bodyCount) == innerIndex,
                "Either this bundle hasn't been initialized yet (and so has unknown count), or it should match the new inner index.");
            for (int i = 0; i < bodyCount; ++i)
            {
                BundleIndexing.GetBundleIndices(bodyIndices[i], out var bodyBundleIndex, out var bodyInnerIndex);
                //Body references, by convention, are stored in bundle-inner-bundle-inner interleaved order.
                Unsafe.Add(ref targetLane, i * stride) = bodyBundleIndex;
                Unsafe.Add(ref targetLane, i * stride + Vector<int>.Count) = bodyInnerIndex;
            }
            //When adding a lane to a body references struct, it is always at the end. So, the count is one beyond the inner index we just set.
            Unsafe.Add(ref start, stride * bodyCount) = innerIndex + 1;
            Debug.Assert(Unsafe.Add(ref start, stride * bodyCount) <= Vector<int>.Count && Unsafe.Add(ref start, stride * bodyCount) >= 0,
                "If the inner index was valid, the resulting bundle count will be within the bundle limits.");

#if DEBUG
            for (int i = 0; i < Unsafe.Add(ref start, stride * bodyCount) - 1; ++i)
            {
                var aIndex = (Unsafe.Add(ref start, i) << BundleIndexing.VectorShift) | Unsafe.Add(ref start, i + Vector<int>.Count);
                var bIndex = (Unsafe.Add(ref start, i + 2 * Vector<int>.Count) << BundleIndexing.VectorShift) | Unsafe.Add(ref start, i + 3 * Vector<int>.Count);
                for (int j = i + 1; j < Unsafe.Add(ref start, stride * bodyCount); ++j)
                {
                    var aIndex2 = (Unsafe.Add(ref start, j) << BundleIndexing.VectorShift) | Unsafe.Add(ref start, j + Vector<int>.Count);
                    var bIndex2 = (Unsafe.Add(ref start, j + 2 * Vector<int>.Count) << BundleIndexing.VectorShift) | Unsafe.Add(ref start, j + 3 * Vector<int>.Count);
                    Debug.Assert(!(
                        aIndex == bIndex || aIndex2 == bIndex2 ||
                        aIndex == aIndex2 || aIndex == bIndex2 ||
                        bIndex == aIndex2 || bIndex == bIndex2),
                        "A bundle should not share any body references. If an add causes redundant body references, something upstream broke.");
                }

            }
#endif
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref int GetLanesInBundleCount(ref TBodyReferences bundle)
        {
            //We assume that the body references struct is organized in memory like Bundle0, Inner0, ... BundleN, InnerN, Count
            //Assuming contiguous storage, Count is then located at start + stride * BodyCount.
            var stride = Vector<int>.Count * 2;
            var bodyCount = Unsafe.SizeOf<TBodyReferences>() / (stride * 4);
            return ref Unsafe.Add(ref Unsafe.As<TBodyReferences, int>(ref bundle), stride * bodyCount);
        }


        protected abstract void RemoveBodyReferences(int bundleIndex, int innerIndex);

        /// <summary>
        /// Allocates room for a constraint without setting up any of the body references information. This should be followed by some other initialization that fills in the 
        /// body references lane and updates the count.
        /// </summary>
        /// <param name="handle">Handle to allocate.</param>
        /// <returns>Index in the type batch allocated for the constraint.</returns>
        private int AllocateShared(int handle)
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
            var index = AllocateShared(handle);
            BundleIndexing.GetBundleIndices(index, out var bundleIndex, out var innerIndex);
            ref var bundle = ref BodyReferences[bundleIndex];
            AddBodyReferencesLane(ref bundle, innerIndex, bodyIndices);
            return index;
        }

        /// <summary>
        /// Allocates a slot in the batch without filling the body references lane for the allocated constraint. The BodyReferences.Count is incremented, though.
        /// </summary>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <param name="allocatedBundleIndex">Bundle index for the constraint.</param>
        /// <param name="allocatedInnerIndex">Index of the constraint within its bundle.</param>
        /// <returns>Index of the slot in the batch.</returns>
        internal int Allocate(int handle, out int allocatedBundleIndex, out int allocatedInnerIndex)
        {
            var index = AllocateShared(handle);
            BundleIndexing.GetBundleIndices(index, out allocatedBundleIndex, out allocatedInnerIndex);
            ref var bundle = ref BodyReferences[allocatedBundleIndex];
            GetLanesInBundleCount(ref bundle) = allocatedInnerIndex + 1;
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
#if DEBUG
            //The Move below overwrites the IndexToHandle, so if we want to use it for debugging, we gotta cache it.
            var removedHandle = IndexToHandle[index];
#endif
            if (index < lastIndex)
            {
                //Need to swap.
                BundleIndexing.GetBundleIndices(index, out var targetBundleIndex, out var targetInnerIndex);
                Move(sourceBundleIndex, sourceInnerIndex, lastIndex, targetBundleIndex, targetInnerIndex, index, handlesToConstraints);
            }
            //Clear the last slot's accumulated impulse regardless of whether a swap takes place. This avoids new constraints getting a weird initial guess.
            GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref AccumulatedImpulses[sourceBundleIndex], sourceInnerIndex);
            RemoveBodyReferences(sourceBundleIndex, sourceInnerIndex);

#if DEBUG
            //While it's not necessary to clear these, it can be useful for debugging if any accesses of the old position (that are not refilled immediately)
            //result in some form of index error later upon invalid usage.
            handlesToConstraints[removedHandle].BatchIndex = -1;
            handlesToConstraints[removedHandle].IndexInTypeBatch = -1;
            handlesToConstraints[removedHandle].TypeId = -1;
            IndexToHandle[lastIndex] = -1;
#endif
        }

        /// <summary>
        /// Moves a constraint from one ConstraintBatch's TypeBatch to another ConstraintBatch's TypeBatch of the same type.
        /// </summary>
        /// <param name="sourceBatch">Batch that owns the type batch that is the source of the constraint transfer.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to move in the current type batch.</param>
        /// <param name="solver">Solver that owns the batches.</param>
        /// <param name="bodies">Bodies set that owns all the constraint's bodies.</param>
        /// <param name="targetBatchIndex">Index of the ConstraintBatch in the solver to copy the constraint into.</param>
        public unsafe override void TransferConstraint(ConstraintBatch sourceBatch, int indexInTypeBatch, Solver solver, Bodies bodies, int targetBatchIndex)
        {
            //Note that the following does some redundant work. It's technically possible to do better than this, but it requires bypassing a lot of bookkeeping.
            //It's not exactly trivial to keep everything straight, especially over time- it becomes a maintenance nightmare.
            //So instead, given that compressions should generally be extremely rare (relatively speaking) and highly deferrable, we'll accept some minor overhead.
            int bodiesPerConstraint = BodiesPerConstraint;
            var bodyHandles = stackalloc int[BodiesPerConstraint];
            var bodyHandleCollector = new ConstraintBodyHandleCollector(bodies, bodyHandles);
            EnumerateConnectedBodyIndices(indexInTypeBatch, ref bodyHandleCollector);
            var targetBatch = solver.Batches.Elements[targetBatchIndex];
            //Allocate a spot in the new batch. Note that it does not change the Handle->Constraint mapping in the Solver; that's important when we call Solver.Remove below.
            var constraintHandle = IndexToHandle[indexInTypeBatch];
            targetBatch.Allocate(constraintHandle, ref bodyHandles[0], bodiesPerConstraint, bodies, solver.TypeBatchAllocation, typeId, out var newReference);

            //This cast is pretty gross looking, but guaranteed to work by virtue of the typeid registrations.
            var targetTypeBatch = (TypeBatch<TBodyReferences, TPrestepData, TProjection, TAccumulatedImpulse>)newReference.TypeBatch;
            BundleIndexing.GetBundleIndices(newReference.IndexInTypeBatch, out var targetBundle, out var targetInner);
            BundleIndexing.GetBundleIndices(indexInTypeBatch, out var sourceBundle, out var sourceInner);
            //We don't pull a description or anything from the old constraint. That would require having a unique mapping from constraint to 'full description'. 
            //Instead, we just directly copy from lane to lane.
            //Note that we leave out the runtime generated bits- they'll just get regenerated.
            GatherScatter.CopyLane(ref BodyReferences[sourceBundle], sourceInner, ref targetTypeBatch.BodyReferences[targetBundle], targetInner);
            GatherScatter.CopyLane(ref PrestepData[sourceBundle], sourceInner, ref targetTypeBatch.PrestepData[targetBundle], targetInner);
            GatherScatter.CopyLane(ref AccumulatedImpulses[sourceBundle], sourceInner, ref targetTypeBatch.AccumulatedImpulses[targetBundle], targetInner);

            //Now we can get rid of the old allocation.
            ValidateBundleCounts(); //note that this validation has to come before the removal, since the removal will mutate this type batch!
            solver.Remove(constraintHandle); //This is safe because the old handle->constraint mapping has not yet been changed. It does do a redundant lookup, though.

            //Don't forget to keep the solver's pointers consistent! We bypassed the usual add procedure, so the solver hasn't been notified yet.
            ref var constraintLocation = ref solver.HandlesToConstraints[constraintHandle];
            constraintLocation.BatchIndex = targetBatchIndex;
            constraintLocation.IndexInTypeBatch = newReference.IndexInTypeBatch;
            constraintLocation.TypeId = typeId;
            targetTypeBatch.ValidateBundleCounts();

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
