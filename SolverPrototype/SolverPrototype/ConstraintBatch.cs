using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Contains a set of constraints which share no body references.
    /// </summary>
    public class ConstraintBatch
    {
        internal BatchReferencedHandles Handles;
        public int[] TypeIndexToTypeBatchIndex;
        public QuickList<TypeBatch> TypeBatches;

        public ConstraintBatch(int initialReferencedHandlesEstimate = 128 * 64, int initialTypeCountEstimate = 32)
        {
            Handles = new BatchReferencedHandles((initialReferencedHandlesEstimate >> 6) + ((initialReferencedHandlesEstimate & 63) > 0 ? 1 : 0));
            ResizeTypeMap(initialTypeCountEstimate);
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

        /// <summary>
        /// Gets a type batch in the batch matching the given type.
        /// Requires that there exists at least one constraint in the type batch.
        /// </summary>
        /// <typeparam name="T">Type of the batch to grab.</typeparam>
        /// <returns>TypeBatch instance associated with the given type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T GetTypeBatch<T>() where T : TypeBatch
        {
            var typeBatchIndex = TypeIndexToTypeBatchIndex[ConstraintTypeIds.GetId<T>()];
            var typeBatch = TypeBatches.Elements[typeBatchIndex];
            Debug.Assert(typeof(T) == TypeBatches.Elements[typeBatchIndex].GetType(), "If the type batch we have stored for this index isn't of the expected type, then something is broken.");
            return Unsafe.As<TypeBatch, T>(ref typeBatch);
        }
        /// <summary>
        /// Gets a type batch in the batch matching the given type id.
        /// Requires that there exists at least one constraint in the type batch.
        /// </summary>
        /// <param name="typeId">Id of the TypeBatch's type to retrieve.</param>
        /// <returns>TypeBatch instance associated with the given type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TypeBatch GetTypeBatch(int typeId)
        {
            var typeBatchIndex = TypeIndexToTypeBatchIndex[typeId];
            return TypeBatches.Elements[typeBatchIndex];
        }

        T CreateNewTypeBatch<T>(int typeId, TypeBatchAllocation typeBatchAllocation) where T : TypeBatch, new()
        {
            var batch = typeBatchAllocation.Take<T>();
            //TODO: should pass an allocator associated with the TypeBatchAllocation into the initializer rather than using the static pools; helps avoid contention cross-simulation.
            batch.Initialize(typeBatchAllocation[typeId]);
            TypeBatches.Add(batch);
            return batch;
        }
        public void Allocate<T>(int handle, ref int bodyReferences, TypeBatchAllocation typeBatchAllocation, out int typeId, out ConstraintReference<T> constraintPointer) where T : TypeBatch, new()
        {        
            typeId = ConstraintTypeIds.GetId<T>();

            if (typeId >= TypeIndexToTypeBatchIndex.Length)
            {
                ResizeTypeMap(1 << BufferPool.GetPoolIndex(typeId));
                TypeIndexToTypeBatchIndex[typeId] = TypeBatches.Count;
                constraintPointer.TypeBatch = CreateNewTypeBatch<T>(typeId, typeBatchAllocation);
            }
            else
            {
                ref var typeBatchIndex = ref TypeIndexToTypeBatchIndex[typeId];
                if (typeBatchIndex == -1)
                {
                    typeBatchIndex = TypeBatches.Count;
                    constraintPointer.TypeBatch = CreateNewTypeBatch<T>(typeId, typeBatchAllocation);
                }
                else
                {
                    Debug.Assert(typeof(T) == TypeBatches.Elements[typeBatchIndex].GetType());
                    constraintPointer.TypeBatch = Unsafe.As<T>(TypeBatches.Elements[typeBatchIndex]);
                }
            }
            constraintPointer.IndexInTypeBatch = constraintPointer.TypeBatch.Allocate(handle, ref bodyReferences);
            //TODO: We could adjust the typeBatchAllocation capacities in response to the allocated index.
            //If it exceeds the current capacity, we could ensure the new size is still included.
            //The idea here would be to avoid resizes later by ensuring that the historically encountered size is always used to initialize.
            //This isn't necessarily beneficial, though- often, higher indexed batches will contain smaller numbers of constraints, so allocating a huge number
            //of constraints into them is very low value. You may want to be a little more clever about the heuristic. Either way, only bother with this once there is 
            //evidence that typebatch resizes are ever a concern. This will require frame spike analysis, not merely average timings.
            //(While resizes will definitely occur, remember that it only really matters for *new* type batches- 
            //and it is rare that a new type batch will be created that actually needs to be enormous.)
        }

        public void Remove<T>(int indexInTypeBatch, ConstraintLocation[] handlesToConstraints, TypeBatchAllocation typeBatchAllocation) where T : TypeBatch
        {
            var constraintTypeId = ConstraintTypeIds.GetId<T>();
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");

            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            var typeBatch = TypeBatches.Elements[typeBatchIndex];
            typeBatch.Remove(indexInTypeBatch, handlesToConstraints);
            if (typeBatch.ConstraintCount == 0)
            {
                TypeIndexToTypeBatchIndex[constraintTypeId] = -1;
                TypeBatches.FastRemoveAt(typeBatchIndex);
                if (typeBatchIndex < TypeBatches.Count)
                {
                    //If we swapped anything into the removed slot, we should update the type index to type batch mapping.
                    TypeIndexToTypeBatchIndex[constraintTypeId] = typeBatchIndex;
                }
                typeBatchAllocation.Return(typeBatch);

            }

        }

    }
}
