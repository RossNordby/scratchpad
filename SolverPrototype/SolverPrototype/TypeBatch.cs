using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public abstract class TypeBatch
    {
        //TODO: Having this in the base class actually complicates the implementation of some special constraint types. Consider an 'articulation' subsolver that involves
        //N bodies, for N > Vector<float>.Count * 2. You may want to do SIMD internally in such a case, so there would be no 'bundles' at this level. Worry about that later.
        protected int bundleCount;
        public int BundleCount => bundleCount;
        protected int constraintCount;
        public int ConstraintCount => constraintCount;

        /// <summary>
        /// The handles for the constraints in this type batch.
        /// </summary>
        public int[] Handles;

        public abstract int Allocate(int handle);
        public abstract void Remove(int index, ConstraintLocation[] handlesToConstraints);

        public abstract void EnumerateConnectedBodyIndices<TEnumerator>(int indexInTypeBatch, ref TEnumerator enumerator) where TEnumerator : IForEach<int>;
        public abstract void UpdateForBodyMemoryMove(int indexInTypeBatch, int bodyIndexInConstraint, int newBodyLocation);

        public abstract void Scramble(Random random, ConstraintLocation[] handlesToConstraints);

        /// <summary>
        /// Sorts a subset of constraints in the type batch according to the location of bodies in memory. The goal is to maximize cache coherence.
        /// </summary>
        /// <param name="startIndex">Start of the sorting region.</param>
        /// <param name="count">Number of constraints to sort.</param>
        /// <param name="handlesToConstraints">The handle to constraint mapping used by the solver that needs to be updated in response to swaps.</param>
        public abstract void SortByBodyLocation(int startIndex, int count, ConstraintLocation[] handlesToConstraints);

        public abstract void Initialize();
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

    }
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


        public static int InitialCapacityInBundles = 128;

        static void IncreaseSize<T>(ref T[] array)
        {
            //Out of space. Need to resize.
            var old = array;
            array = BufferPools<T>.Locking.Take(array.Length << 1);
            Array.Copy(old, array, old.Length);
            BufferPools<T>.Locking.Return(old);
        }


        /// <summary>
        /// Allocates a slot in the batch.
        /// </summary>
        /// <param name="handle">Handle of the constraint to allocate. Establishes a link from the allocated constraint to its handle.</param>
        /// <returns>Index of the slot in the batch.</returns>
        public override int Allocate(int handle)
        {
            Debug.Assert(Projection != null, "Should initialize the batch before allocating anything from it.");
            if (constraintCount == Projection.Length)
            {
                IncreaseSize(ref BodyReferences);
                //TODO: So long as increase size is using a non-clearing pool, we need to clear the body references to avoid pulling old counts. We rely on the counts.
                //Would be a good idea to change this- it would be easy and cheap to clear the count every time a new bundle is created.
                Array.Clear(BodyReferences, BundleCount, BodyReferences.Length - BundleCount);
                IncreaseSize(ref PrestepData);
                IncreaseSize(ref Projection);
                IncreaseSize(ref AccumulatedImpulses);
                IncreaseSize(ref Handles);
            }
            var index = constraintCount++;
            Handles[index] = handle;
            if ((constraintCount & BundleIndexing.VectorMask) == 1)
                ++bundleCount;
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
            Handles[targetIndex] = sourceHandle;
            handlesToConstraints[sourceHandle].IndexInTypeBatch = targetIndex;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(int sourceBundle, int sourceInner, int sourceIndex, int targetBundle, int targetInner, int targetIndex,
             ConstraintLocation[] handlesToConstraints)
        {
            Move(ref BodyReferences[sourceBundle], ref PrestepData[sourceBundle], ref AccumulatedImpulses[sourceBundle], sourceInner, Handles[sourceIndex],
                targetBundle, targetInner, targetIndex, handlesToConstraints);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        protected void Move(int sourceIndex, int targetIndex,
            ConstraintLocation[] handlesToConstraints)
        {
            BundleIndexing.GetBundleIndices(sourceIndex, out var sourceBundle, out var sourceInner);
            BundleIndexing.GetBundleIndices(targetIndex, out var targetBundle, out var targetInner);
            Move(ref BodyReferences[sourceBundle], ref PrestepData[sourceBundle], ref AccumulatedImpulses[sourceBundle], sourceInner, Handles[sourceIndex],
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
                aHandle = Handles[a];

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
                Handles[index] = Handles[lastIndex];
            }
            //Clear the last slot's accumulated impulse regardless of whether a swap takes place. This avoids new constraints getting a weird initial guess.
            GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref AccumulatedImpulses[sourceBundleIndex], sourceInnerIndex);
        }

        public override void Initialize()
        {
            Projection = BufferPools<TProjection>.Locking.Take(InitialCapacityInBundles);
            BodyReferences = BufferPools<TBodyReferences>.Locking.Take(InitialCapacityInBundles);
            //TODO: So long as increase size is using a non-clearing pool, we need to clear the body references to avoid pulling old counts. We rely on the counts.
            //Would be a good idea to change this- it would be easy and cheap to clear the count every time a new bundle is created.
            Array.Clear(BodyReferences, 0, BodyReferences.Length);
            PrestepData = BufferPools<TPrestepData>.Locking.Take(InitialCapacityInBundles);
            AccumulatedImpulses = BufferPools<TAccumulatedImpulse>.Locking.Take(InitialCapacityInBundles);
            Handles = BufferPools<int>.Locking.Take(InitialCapacityInBundles * Vector<float>.Count);
        }

        public override void Reset()
        {
            BufferPools<TProjection>.Locking.Return(Projection);
            BufferPools<TBodyReferences>.Locking.Return(BodyReferences);
            BufferPools<TPrestepData>.Locking.Return(PrestepData);
            BufferPools<TAccumulatedImpulse>.Locking.Return(AccumulatedImpulses);
            BufferPools<int>.Locking.Return(Handles);
            Projection = null;
            BodyReferences = null;
            AccumulatedImpulses = null;
            AccumulatedImpulses = null;
            Handles = null;
        }



    }
}
