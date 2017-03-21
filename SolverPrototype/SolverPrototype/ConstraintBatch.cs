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
        internal BatchReferencedHandles BodyHandles;
        public int[] TypeIndexToTypeBatchIndex;
        public QuickList<TypeBatch> TypeBatches;

        public ConstraintBatch(int initialReferencedHandlesEstimate = 128 * 64, int initialTypeCountEstimate = 32)
        {
            BodyHandles = new BatchReferencedHandles(initialReferencedHandlesEstimate);
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

        TypeBatch CreateNewTypeBatch(int typeId, TypeBatchAllocation typeBatchAllocation)
        {
            var batch = typeBatchAllocation.Take(typeId);
            TypeBatches.Add(batch);
            return batch;
        }

        /// <summary>
        /// Gets whether the batch could hold the specified body handles.
        /// </summary>
        /// <param name="constraintBodyHandles">List of body handles to check for in the batch.</param>
        /// <param name="constraintBodyHandleCount">Number of bodies referenced by the constraint.</param>
        /// <returns>True if the body handles are not already present in the batch, false otherwise.</returns>
        public unsafe bool CanFit(ref int constraintBodyHandles, int constraintBodyHandleCount)
        {
            for (int i = 0; i < constraintBodyHandleCount; ++i)
            {
                var bodyHandle = Unsafe.Add(ref constraintBodyHandles, i);
                if (BodyHandles.Contains(bodyHandle))
                {
                    return false;
                }
            }
            return true;
        }

        public unsafe void Allocate(int handle, ref int bodyHandles, int bodyCount, Bodies bodies, TypeBatchAllocation typeBatchAllocation, int typeId, out ConstraintReference reference)
        {
            Debug.Assert(CanFit(ref bodyHandles, bodyCount));
            //Add all the constraint's body handles to the batch we found (or created) to block future references to the same bodies.
            //Also, convert the handle into a memory index. Constraints store a direct memory reference for performance reasons.
            var bodyIndices = stackalloc int[bodyCount];
            for (int j = 0; j < bodyCount; ++j)
            {
                var bodyHandle = Unsafe.Add(ref bodyHandles, j);
                BodyHandles.Add(bodyHandle);
                bodyIndices[j] = bodies.HandleToIndex[bodyHandle];
            }
            if (typeId >= TypeIndexToTypeBatchIndex.Length)
            {
                ResizeTypeMap(1 << BufferPool.GetPoolIndex(typeId));
                TypeIndexToTypeBatchIndex[typeId] = TypeBatches.Count;
                reference.TypeBatch = CreateNewTypeBatch(typeId, typeBatchAllocation);
            }
            else
            {
                ref var typeBatchIndex = ref TypeIndexToTypeBatchIndex[typeId];
                if (typeBatchIndex == -1)
                {
                    typeBatchIndex = TypeBatches.Count;
                    reference.TypeBatch = CreateNewTypeBatch(typeId, typeBatchAllocation);
                }
                else
                {
                    Debug.Assert(ConstraintTypeIds.GetType(typeId) == TypeBatches.Elements[typeBatchIndex].GetType());
                    reference.TypeBatch = TypeBatches.Elements[typeBatchIndex];
                }
            }
            reference.IndexInTypeBatch = reference.TypeBatch.Allocate(handle, bodyIndices);
            //TODO: We could adjust the typeBatchAllocation capacities in response to the allocated index.
            //If it exceeds the current capacity, we could ensure the new size is still included.
            //The idea here would be to avoid resizes later by ensuring that the historically encountered size is always used to initialize.
            //This isn't necessarily beneficial, though- often, higher indexed batches will contain smaller numbers of constraints, so allocating a huge number
            //of constraints into them is very low value. You may want to be a little more clever about the heuristic. Either way, only bother with this once there is 
            //evidence that typebatch resizes are ever a concern. This will require frame spike analysis, not merely average timings.
            //(While resizes will definitely occur, remember that it only really matters for *new* type batches- 
            //and it is rare that a new type batch will be created that actually needs to be enormous.)
        }
        

        unsafe struct BodyHandleRemover : IForEach<int>
        {
            public Bodies Bodies;
            public ConstraintBatch Batch;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public BodyHandleRemover(Bodies bodies, ConstraintBatch batch)
            {
                Bodies = bodies;
                Batch = batch;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int bodyIndex)
            {
                Batch.BodyHandles.Remove(Bodies.IndexToHandle[bodyIndex]);
            }
        }


        public unsafe void Remove(int constraintTypeId, int indexInTypeBatch, Bodies bodies, ConstraintLocation[] handlesToConstraints, TypeBatchAllocation typeBatchAllocation)
        {
            Debug.Assert(TypeIndexToTypeBatchIndex[constraintTypeId] >= 0, "Type index must actually exist within this batch.");

            var typeBatchIndex = TypeIndexToTypeBatchIndex[constraintTypeId];
            var typeBatch = TypeBatches.Elements[typeBatchIndex];
            //Before we remove the constraint, we should locate the set the body indices referenced by the constraint and convert them into handles so that
            //they can be removed from the constraint batch's body handle set.
            var bodiesPerConstraint = typeBatch.BodiesPerConstraint;
            var handleRemover = new BodyHandleRemover(bodies, this);
            typeBatch.EnumerateConnectedBodyIndices(indexInTypeBatch, ref handleRemover);

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
                typeBatchAllocation.Return(typeBatch, constraintTypeId);

            }

        }

    }
}
