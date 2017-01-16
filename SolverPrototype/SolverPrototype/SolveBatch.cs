using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public abstract class SolveBatch
    {
        protected int bundleCount;
        public int BundleCount => bundleCount;
        protected int constraintCount;
        public int ConstraintCount => constraintCount;

        public abstract int Allocate();
        public abstract void Remove(int index);

        public abstract void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle);
        public abstract void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle);
    }
    /// <summary>
    /// Handles the solve iterations of a bunch of 1DOF two body inequality constraints.
    /// </summary>
    public class SolveBatch<T> : SolveBatch
    {
        public T[] IterationData;

        public static int InitialCapacity = 128;
        public SolveBatch()
        {
            IterationData = BufferPools<T>.Locking.Take(InitialCapacity);
        }

        /// <summary>
        /// Allocates a slot in the batch.
        /// </summary>
        /// <returns>Index of the slot in the batch.</returns>
        public override int Allocate()
        {
            if (constraintCount == IterationData.Length)
            {
                //Out of space. Need to resize.
                var old = IterationData;
                IterationData = BufferPools<T>.Locking.Take(IterationData.Length << 1);
                Array.Copy(old, IterationData, old.Length);
                BufferPools<T>.Locking.Return(old);
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
            if (index == lastIndex)
            {
                //No swap required; it's the last element.
                return;
            }
            //Need to swap.
            BundleIndexing.GetBundleIndices(index, out var targetBundleIndex, out var targetInnerIndex);
            BundleIndexing.GetBundleIndices(lastIndex, out var sourceBundleIndex, out var sourceInnerIndex);
            GatherScatter.CopyLane(ref IterationData[sourceBundleIndex], sourceInnerIndex, ref IterationData[targetBundleIndex], targetInnerIndex);
        }
        

        public override void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            for (int i = startBundle; i < endBundle; ++i)
            {
                Inequality2Body1DOF.WarmStart(bodyVelocities, ref IterationData[i]);
            }
        }
        public override void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            for (int i = startBundle; i < endBundle; ++i)
            {
                Inequality2Body1DOF.Solve(bodyVelocities, ref IterationData[i]);
            }
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
