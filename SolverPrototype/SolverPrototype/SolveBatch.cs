using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public abstract class TypeBatch
    {
        protected int bundleCount;
        public int BundleCount => bundleCount;
        protected int constraintCount;
        public int ConstraintCount => constraintCount;

        public abstract int Allocate();
        public abstract void Remove(int index);

        public abstract void Initialize();
        public abstract void Reset();

        public abstract void Prestep(BodyInertias[] bodyInertias, float dt, float inverseDt, int startBundle, int endBundle);
        public abstract void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle);
        public abstract void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle);
    }
    public abstract class TypeBatch<TBodyReferences, TPrestepData, TIterationData, TAccumulatedImpulse> : TypeBatch
    {
        public TBodyReferences[] BodyReferences;
        public TPrestepData[] PrestepData;
        //Technically, the iteration data does not need to persist outside of the scope of the solve. We let it persist for simplicity- it does not take much space.
        /// <summary>
        /// Data required by the WarmStart and SolveIteration. Note that this data is conceptually ephemeral- external users should not depend upon it outside of the solver's execution.
        /// (At the moment, it does persist, but it becomes unreliable when constraints are removed, and the implementation reserves the right to make it completely temporary.)
        /// </summary>
        protected TIterationData[] IterationData;
        public TAccumulatedImpulse[] AccumulatedImpulses;

        public static int InitialCapacity = 128;

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
        /// <returns>Index of the slot in the batch.</returns>
        public override int Allocate()
        {
            Debug.Assert(IterationData != null, "Should initialize the batch before allocating anything from it.");
            if (constraintCount == IterationData.Length)
            {
                IncreaseSize(ref PrestepData);
                IncreaseSize(ref IterationData);
            }
            var index = constraintCount++;
            if ((constraintCount & BundleIndexing.VectorMask) == 1)
                ++bundleCount;
            return index;
        }

        /// <summary>
        /// Removes a constraint from the batch.
        /// </summary>
        /// <param name="index">Index of the constraint to remove.</param>
        public override void Remove(int index)
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
                //Note that we do NOT copy the iteration data. It is regenerated each frame from scratch. 
                //We may later decide that this is silly because someone might rely on it, but... it seems very unlikely. 
                //Try to stop people from relying on it, and see if anyone ever complains.
                GatherScatter.CopyLane(ref BodyReferences[sourceBundleIndex], sourceInnerIndex, ref BodyReferences[targetBundleIndex], targetInnerIndex);
                GatherScatter.CopyLane(ref PrestepData[sourceBundleIndex], sourceInnerIndex, ref PrestepData[targetBundleIndex], targetInnerIndex);
                GatherScatter.CopyLane(ref AccumulatedImpulses[sourceBundleIndex], sourceInnerIndex, ref AccumulatedImpulses[targetBundleIndex], targetInnerIndex);
            }
            //Clear the last slot's accumulated impulse regardless of whether a swap takes place. This avoids new constraints getting a weird initial guess.
            GatherScatter.ClearLane<TAccumulatedImpulse, float>(ref AccumulatedImpulses[sourceBundleIndex], sourceInnerIndex);
        }

        public override void Initialize()
        {
            IterationData = BufferPools<TIterationData>.Locking.Take(InitialCapacity);
        }

        public override void Reset()
        {
            BufferPools<TIterationData>.Locking.Return(IterationData);
            IterationData = null;
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
}
