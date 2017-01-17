using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Contains a set of constraints which share no body references.
    /// </summary>
    class ConstraintBatch
    {
        public BatchReferencedHandles Handles;
        public int[] TypeIndexToTypeBatchIndex;
        public QuickList<TypeBatch> TypeBatches;

        public ConstraintBatch()
        {
            Handles = new BatchReferencedHandles(128);
            ResizeTypeMap(16);
            TypeBatches = new QuickList<TypeBatch>(new PassthroughBufferPool<TypeBatch>());
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

        public T GetTypeBatch<T>() where T : TypeBatch
        {
            var typeBatchIndex = TypeIndexToTypeBatchIndex[ConstraintTypeIds.GetId<T>()];
            var typeBatch = TypeBatches.Elements[typeBatchIndex];
            Debug.Assert(typeof(T) == TypeBatches.Elements[typeBatchIndex].GetType(), "If the type batch we have stored for this index isn't of the expected type, then something is broken.");
            return Unsafe.As<TypeBatch, T>(ref typeBatch);
        }


        public int Allocate<T>() where T : TypeBatch, new()
        {
            var typeId = ConstraintTypeIds.GetId<T>();
            TypeBatch typeBatch;
            if (typeId >= TypeIndexToTypeBatchIndex.Length)
            {
                ResizeTypeMap(1 << BufferPool.GetPoolIndex(typeId));
                TypeIndexToTypeBatchIndex[typeId] = TypeBatches.Count;
                TypeBatches.Add(typeBatch = ConstraintTypeIds.Take<T>());
            }
            else
            {
                ref var typeBatchIndex = ref TypeIndexToTypeBatchIndex[typeId];
                if (typeBatchIndex == -1)
                {
                    typeBatchIndex = TypeBatches.Count;
                    TypeBatches.Add(typeBatch = ConstraintTypeIds.Take<T>());
                }
                else
                {
                    typeBatch = TypeBatches.Elements[typeBatchIndex];
                }
            }
            return typeBatch.Allocate();
        }

        public void Remove<T>(int indexInTypeBatch) where T : TypeBatch
        {
            var constraintTypeId = ConstraintTypeIds.GetId<T>();
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");

            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            var typeBatch = TypeBatches.Elements[typeBatchIndex];
            typeBatch.Remove(indexInTypeBatch);
            if (typeBatch.ConstraintCount == 0)
            {
                TypeIndexToTypeBatchIndex[constraintTypeId] = -1;
                TypeBatches.FastRemoveAt(typeBatchIndex);
                if (typeBatchIndex < TypeBatches.Count)
                {
                    //If we swapped anything into the removed slot, we should update the type index to type batch mapping.
                    TypeIndexToTypeBatchIndex[constraintTypeId] = typeBatchIndex;
                }
                ConstraintTypeIds.Return(typeBatch);

            }

        }


    }
}
