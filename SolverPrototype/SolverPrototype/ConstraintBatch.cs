using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;

namespace SolverPrototype
{
    /// <summary>
    /// Contains a set of constraints which share no body references.
    /// </summary>
    class ConstraintBatch
    {
        public BatchReferencedHandles Handles;
        public int[] TypeIndexToTypeBatchIndex;
        public QuickList<ConstraintTypeBatch> TypeBatches;

        public ConstraintBatch()
        {
            Handles = new BatchReferencedHandles(128);
            ResizeTypeMap(16);
            TypeBatches = new QuickList<ConstraintTypeBatch>(new PassthroughBufferPool<ConstraintTypeBatch>());
        }

        void ResizeTypeMap(int newSize)
        {
            var oldLength = TypeIndexToTypeBatchIndex == null ? 0 : TypeIndexToTypeBatchIndex.Length;
            Array.Resize(ref TypeIndexToTypeBatchIndex, newSize);
            for (int i = oldLength; i < TypeIndexToTypeBatchIndex.Length; ++i)
            {
                TypeIndexToTypeBatchIndex[i] = -1;
            }
        }

        public void Add<T>(ref T constraint) where T : IConstraintDescription
        {
            ConstraintTypeBatch typeBatch;
            if (constraint.ConstraintTypeId >= TypeIndexToTypeBatchIndex.Length)
            {
                ResizeTypeMap(1 << BufferPool.GetPoolIndex(constraint.ConstraintTypeId));
                TypeIndexToTypeBatchIndex[constraint.ConstraintTypeId] = TypeBatches.Count;
                TypeBatches.Add(typeBatch = ConstraintTypeBatch.TypeBatchPools[constraint.ConstraintTypeId].LockingTake());
            }
            else
            {
                ref var typeBatchIndex = ref TypeIndexToTypeBatchIndex[constraint.ConstraintTypeId];
                if (typeBatchIndex == -1)
                {
                    typeBatchIndex = TypeBatches.Count;
                    TypeBatches.Add(typeBatch = ConstraintTypeBatch.TypeBatchPools[constraint.ConstraintTypeId].LockingTake());
                }
                else
                {
                    typeBatch = TypeBatches.Elements[typeBatchIndex];
                }
            }
            typeBatch.Add(ref constraint);
        }

        public void Remove(int constraintTypeId, int constraintHandle)
        {
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");

            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            var typeBatch = TypeBatches.Elements[typeBatchIndex];
            typeBatch.Remove(constraintHandle);
            if (typeBatch.ConstraintCount == 0)
            {
                TypeIndexToTypeBatchIndex[constraintTypeId] = -1;
                TypeBatches.FastRemoveAt(typeBatchIndex);
                if (typeBatchIndex < TypeBatches.Count)
                {
                    //If we swapped anything into the removed slot, we should update the type index to type batch mapping.
                    TypeIndexToTypeBatchIndex[TypeBatches.Elements[typeBatchIndex].ConstraintTypeIndex] = typeBatchIndex;
                }
                ConstraintTypeBatch.TypeBatchPools[constraintTypeId].LockingReturn(typeBatch);

            }

        }


    }
}
