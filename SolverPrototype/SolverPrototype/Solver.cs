﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;

namespace SolverPrototype
{
    /// <summary>
    /// Direct reference to a particular constraint in a type batch. 
    /// The index may be invalidated by removal of other constraints in the type batch. Removals can occur by user request or by batch compression.
    /// The user is responsible for guaranteeing safe usage. For long term references that potentially span removals, use the constraint's handle.
    /// </summary>
    public struct ConstraintReference
    {
        public TypeBatch TypeBatch;
        public int IndexInTypeBatch;
    }

    public struct ConstraintLocation
    {
        //Note that the type id is included, even though we can extract it from a type parameter.
        //This is required for body memory swap induced reference changes- it is not efficient to include type metadata in the per-body connections,
        //so instead we keep a type id cached.
        //(You could pack these a bit- it's pretty reasonable to say you can't have more than 2^24 constraints of a given type and 2^8 constraint types...
        //It's just not that valuable, until proven otherwise.)
        public int BatchIndex;
        public int TypeId;
        public int IndexInTypeBatch;
    }

    public partial class Solver
    {
        public QuickList<ConstraintBatch, Array<ConstraintBatch>> Batches;

        int iterationCount;
        /// <summary>
        /// Gets or sets the number of solver iterations to compute per call to Update.
        /// </summary>
        public int IterationCount
        {
            get { return iterationCount; }
            set
            {
                if (value < 1)
                {
                    throw new ArgumentException("Iteration count must be positive.");
                }
                iterationCount = value;
            }
        }

        Bodies bodies;

        internal IdPool<Buffer<int>> handlePool;
        BufferPool bufferPool;
        public Buffer<ConstraintLocation> HandleToConstraint;

        public TypeBatchAllocation TypeBatchAllocation { get; private set; }

        /// <summary>
        /// Gets the total number of constraints across all types and batches.
        /// </summary>
        public int ConstraintCount
        {
            get
            {
                int count = 0;
                for (int i = 0; i < Batches.Count; ++i)
                {
                    var batch = Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        count += batch.TypeBatches[j].ConstraintCount;
                    }
                }
                return count;
            }
        }
        /// <summary>
        /// Gets the total number of bundles across all types and batches.
        /// </summary>
        public int BundleCount
        {
            get
            {
                int count = 0;
                for (int i = 0; i < Batches.Count; ++i)
                {
                    var batch = Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        count += batch.TypeBatches[j].BundleCount;
                    }
                }
                return count;
            }
        }

        Action<int> workDelegate;
        const int TypeCountEstimate = 32;
        const int BatchCountEstimate = 32;
        public Solver(Bodies bodies, BufferPool bufferPool, int iterationCount = 5,
            int initialCapacity = 1024,
            int minimumCapacityPerTypeBatch = 64)
        {
            this.iterationCount = iterationCount;
            this.bodies = bodies;
            this.bufferPool = bufferPool;
            IdPool<Buffer<int>>.Create(bufferPool.SpecializeFor<int>(), 128, out handlePool);
            //Note that managed arrays must be used to hold the reference types. It's technically possible to bypass this by completely abandoning inheritance in the typebatches, but
            //that would make a variety of things more annoying to handle. We can make use of just a tiny amount of idiomatic C#-ness. This won't be many references anyway.
            //We also don't bother pooling this stuff, and we don't have an API for preallocating it- because we're talking about a very, very small amount of data.
            //It's not worth the introduced API complexity.
            QuickList<ConstraintBatch, Array<ConstraintBatch>>.Create(new PassthroughArrayPool<ConstraintBatch>(), BatchCountEstimate, out Batches);
            bufferPool.SpecializeFor<ConstraintLocation>().Take(initialCapacity, out HandleToConstraint);
            TypeBatchAllocation = new TypeBatchAllocation(TypeCountEstimate, minimumCapacityPerTypeBatch, bufferPool);
            workDelegate = Work;
        }

        /// <summary>
        /// Gets a direct reference to the constraint associated with a handle.
        /// The reference is temporary; any constraint removals that affect the referenced type batch may invalidate the index.
        /// </summary>
        /// <typeparam name="T">Type of the type batch being referred to.</typeparam>
        /// <param name="handle">Handle index of the constraint.</param>
        /// <param name="reference">Temporary direct reference to the type batch and index in the type batch associated with the constraint handle.
        /// May be invalidated by constraint removals.</param>
        public void GetConstraintReference(int handle, out ConstraintReference reference)
        {
            ref var constraintLocation = ref HandleToConstraint[handle];
            reference.TypeBatch = Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
            reference.IndexInTypeBatch = constraintLocation.IndexInTypeBatch;
        }

        /// <summary>
        /// Allocates a slot in a type batch for a constraint.
        /// </summary>
        /// <typeparam name="T">Type of the TypeBatch to allocate in.</typeparam>
        /// <param name="bodyHandles">Reference to the start of a list of body handles.</param>
        /// <param name="bodyCount">Number of bodies in the body handles list.</param>
        /// <param name="typeId">Id of the TypeBatch type to allocate in.</param>
        /// <param name="reference">Direct reference to the constraint type batch and index in the type batch.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe int Allocate(ref int bodyHandles, int bodyCount, int typeId, out ConstraintReference reference)
        {
            int targetBatchIndex = -1;
            //Find the first batch that references none of the bodies that this constraint needs.
            for (int i = 0; i < Batches.Count; ++i)
            {
                if (Batches[i].CanFit(ref bodyHandles, bodyCount))
                {
                    targetBatchIndex = i;
                    break;
                }
            }
            ConstraintBatch targetBatch;
            if (targetBatchIndex == -1)
            {
                //No batch available. Have to create a new one.
                //Note that we have no explicit pooling. Instead, we just use the array backing the batches list as the pool. 
                //Batches grow and shrink like a stack since the removals only ever occur when an empty batch is in the very last slot.
                //So, all we have to do is check the backing array slot- if there's already a batch there, use it.
                targetBatchIndex = Batches.Count;
                if (Batches.Span.Length > Batches.Count && Batches.Span[targetBatchIndex] != null)
                {
                    //Reusable batch found! 
                    targetBatch = Batches.Span[targetBatchIndex];
                    Debug.Assert(targetBatch.TypeBatches.Count == 0);
                    ++Batches.Count;
                }
                else
                {
                    //No reusable batch found. Create a new one.
                    targetBatch = new ConstraintBatch(bufferPool, bodies.BodyCount, TypeCountEstimate);
                    Batches.Add(targetBatch, new PassthroughArrayPool<ConstraintBatch>());
                }
            }
            else
            {
                targetBatch = Batches[targetBatchIndex];
            }
            var handle = handlePool.Take();
            targetBatch.Allocate(handle, ref bodyHandles, bodyCount, bodies, TypeBatchAllocation, typeId, out reference);

            if (handle >= HandleToConstraint.Length)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, HandleToConstraint.Length * 2, HandleToConstraint.Length);
                Debug.Assert(handle < HandleToConstraint.Length, "Handle indices should never jump by more than 1 slot, so doubling should always be sufficient.");
            }
            HandleToConstraint[handle].IndexInTypeBatch = reference.IndexInTypeBatch;
            HandleToConstraint[handle].TypeId = typeId;
            HandleToConstraint[handle].BatchIndex = targetBatchIndex;
            return handle;
        }

        /// <summary>
        /// Applies a description to a constraint slot.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintReference">Reference of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescription<TDescription>(ref ConstraintReference constraintReference, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            description.ApplyDescription(constraintReference.TypeBatch, bundleIndex, innerIndex);
        }


        /// <summary>
        /// Applies a description to a constraint slot.
        /// </summary>
        /// <typeparam name="TDescription">Type of the description to apply.</typeparam>
        /// <param name="constraintReference">Handle of the constraint being updated.</param>
        /// <param name="description">Description to apply to the slot.</param>
        public void ApplyDescription<TDescription>(int constraintHandle, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            GetConstraintReference(constraintHandle, out var constraintReference);
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            description.ApplyDescription(constraintReference.TypeBatch, bundleIndex, innerIndex);
        }

        /// <summary>
        /// Allocates a constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <typeparam name="TTypeBatch">Type of the TypeBatch to allocate in.</typeparam>
        /// <param name="bodyHandles">Reference to the start of a list of body handles.</param>
        /// <param name="bodyCount">Number of bodies in the body handles list.</param>
        /// <param name="constraintReference">Reference to the allocated slot.</param>
        /// <param name="handle">Allocated constraint handle.</param>
        public void Add<TDescription>(ref int bodyHandles, int bodyCount, ref TDescription description, out int handle)
            where TDescription : IConstraintDescription<TDescription>
        {
            handle = Allocate(ref bodyHandles, bodyCount, description.ConstraintTypeId, out var reference);
            ApplyDescription(ref reference, ref description);

        }

        //This is split out for use by the multithreaded constraint remover.
        internal void RemoveBatchIfEmpty(ConstraintBatch batch, int batchIndex)
        {
            if (batch.TypeBatches.Count == 0)
            {
                //No more constraints exist within the batch; we may be able to get rid of this batch.
                //Merely having no constraints is insufficient. We would really rather not remove a batch if there are batches 'above' it:
                //the handle->constraint mapping involves a batch index. If we removed this batch, it would move every other batch down one step.
                //Which means, in order to retain correctness, we would have to change the batch index on every single constraint in every single batch 
                //of a higher index.

                //That's not feasible in the worst case.

                //Instead, we will only remove the batch if it is the *last* batch. We then rely on deferred batch compression to move constraints into lower
                //batches over time. Since it only handles a limited number of constraints at a time, it doesn't have the same risk of frame hitching.
                //So, even if a low-index batch gets zeroed out, constraints from higher batches will filter down, leaving the highest index constraint potentially empty instead.

                //This is a pretty safe thing to do. It is extremely difficult for a low index batch to end up empty while a higher index batch is still very full.
                //In fact, almost every instance where a batch goes empty will involve the highest index batch. If it isn't, it's going to be very high, and there won't be 
                //many constraints above it. Deferred compression will handle it easily.
                //Note the use of the cached batch index rather than the ref.
                if (batchIndex == Batches.Count - 1)
                {
                    //Note that when we remove an empty batch, it may reveal another empty batch. If that happens, remove the revealed batch(es) too.
                    while (Batches.Count > 0 && Batches[Batches.Count - 1].TypeBatches.Count == 0)
                    {
                        //Note that we do not actually null out the batch slot. It's still there. The backing array of the Batches list acts as a pool. When a new batch is required,
                        //the add function first checks the backing array to see if a batch was already allocated for it. In effect, adding and removing batches behaves like a stack.
                        --Batches.Count;
                    }
                }
            }
        }

        /// <summary>
        /// Removes a constraint from a batch, performing any necessary batch cleanup, but does not return the constraint's handle to the pool.
        /// </summary>
        /// <param name="batchIndex">Index of the batch to remove from.</param>
        /// <param name="typeId">Type id of the constraint to remove.</param>
        /// <param name="indexInTypeBatch">Index of the constraint to remove within its type batch.</param>
        internal void RemoveFromBatch(int batchIndex, int typeId, int indexInTypeBatch)
        {
            var batch = Batches[batchIndex];
            batch.Remove(typeId, indexInTypeBatch, bodies, ref HandleToConstraint, TypeBatchAllocation);
            RemoveBatchIfEmpty(batch, batchIndex);
        }
        /// <summary>
        /// Removes the constraint associated with the given handle. Note that this may invalidate any outstanding direct constraint references (TypeBatch-index pairs)
        /// by reordering the constraints within the TypeBatch subject to removal.
        /// </summary>
        /// <param name="handle">Handle of the constraint to remove from the solver.</param>
        public void Remove(int handle)
        {
            //Note that we don't use a ref var here. Have to be careful; we make use of the constraint location after removal. Direct ref would be invalidated.
            //(Could cache the batch index, but that's splitting some very fine hairs.)
            var constraintLocation = HandleToConstraint[handle];
            RemoveFromBatch(constraintLocation.BatchIndex, constraintLocation.TypeId, constraintLocation.IndexInTypeBatch);
            handlePool.Return(handle, bufferPool.SpecializeFor<int>());
        }

        public void GetDescription<TConstraintDescription, TTypeBatch>(ref ConstraintReference constraintReference, out TConstraintDescription description)
            where TConstraintDescription : IConstraintDescription<TConstraintDescription>
            where TTypeBatch : TypeBatch
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            default(TConstraintDescription).BuildDescription(constraintReference.TypeBatch, bundleIndex, innerIndex, out description);

        }

        public void GetDescription<TConstraintDescription>(int handle, out TConstraintDescription description)
            where TConstraintDescription : IConstraintDescription<TConstraintDescription>
        {
            //Note that the inlining behavior of the BuildDescription function is critical for efficiency here.
            //If the compiler can prove that the BuildDescription function never references any of the instance fields, it will elide the (potentially expensive) initialization.
            //The BuildDescription and ConstraintTypeId members are basically static. It would be nice if C# could express that a little more cleanly with no overhead.
            ref var location = ref HandleToConstraint[handle];
            var dummy = default(TConstraintDescription);
            var typeBatch = Batches[location.BatchIndex].GetTypeBatch(dummy.ConstraintTypeId);
            BundleIndexing.GetBundleIndices(location.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
            dummy.BuildDescription(typeBatch, bundleIndex, innerIndex, out description);

        }



        /// <summary>
        /// Changes the body reference of a constraint in response to a body memory move.
        /// </summary>
        /// <param name="constraintHandle">Handle of the constraint to modify.</param> 
        /// <param name="bodyIndexInConstraint">Index of the moved body in the constraint.</param>
        /// <param name="newBodyLocation">Memory index that the moved body now inhabits.</param>
        public void UpdateForBodyMemoryMove(int constraintHandle, int bodyIndexInConstraint, int newBodyLocation)
        {
            //Note that this function requires scanning the bodies in the constraint. This will tend to be fine since the vast majority of constraints have no more than 2 bodies.
            //While it's possible to store the index of the body in the constraint to avoid this scan, storing that information requires collecting that information on add.
            //That's not impossible by any means, but consider that this function will tend to be called in a deferred way- we have control over how many cache optimizations
            //we perform. We do not, however, have any control over how many adds must be performed. Those must be performed immediately for correctness.
            //In other words, doing a little more work here can reduce the overall work required, in addition to simplifying the storage requirements.
            ref var constraintLocation = ref HandleToConstraint[constraintHandle];
            //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
            //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
            //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
            Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId).UpdateForBodyMemoryMove(constraintLocation.IndexInTypeBatch, bodyIndexInConstraint, newBodyLocation);
        }

        /// <summary>
        /// Enumerates the set of body indices associated with a constraint in order of their references within the constraint.
        /// </summary>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to use.</param>
        internal void EnumerateConnectedBodyIndices<TEnumerator>(int constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var constraintLocation = ref HandleToConstraint[constraintHandle];
            //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
            //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
            //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
            Batches[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId).EnumerateConnectedBodyIndices(constraintLocation.IndexInTypeBatch, ref enumerator);
        }


        public void Update(float dt)
        {
            for (int i = 0; i < Batches.Count; ++i)
            {
                var batch = Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches[j].Prestep(bodies, dt);
                }
            }
            //TODO: May want to consider executing warmstart immediately following the prestep. Multithreading can't do that, so there could be some bitwise differences introduced.
            //On the upside, it would make use of cached data.
            for (int i = 0; i < Batches.Count; ++i)
            {
                var batch = Batches[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches[j].WarmStart(ref bodies.Velocities);
                }
            }
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < Batches.Count; ++i)
                {
                    var batch = Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        batch.TypeBatches[j].SolveIteration(ref bodies.Velocities);
                    }
                }
            }
        }

        //Note that none of these affect the constraint batch estimates or type batch estimates. The assumption is that those are too small to bother with.
        //In the worst case you might see a couple of kilobytes. The reason why these functions exist is to deal with the potential many *megabytes* worth of constraint and body buffers.
        //Maybe something weird happens where this assumption is invalidated later, but I doubt it. There is a cost in API complexity to support it, so we don't.

        /// <summary>
        /// Removes all objects from the solver without returning any resources to the memory pool.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Clear(TypeBatchAllocation);
            }
            //Note that the 'pooled' batches are stored in place. By changing the count, we move all existing batches into the pool.
            Batches.Count = 0;
            handlePool.Clear();
        }

        public void EnsureCapacity(int bodiesCount, int constraintCount, int constraintsPerTypeBatch)
        {
            if (!Batches.Span.Allocated)
            {
                //This solver instance was disposed, so we need to explicitly reconstruct the batches array.
                QuickList<ConstraintBatch, Array<ConstraintBatch>>.Create(new PassthroughArrayPool<ConstraintBatch>(), BatchCountEstimate, out Batches);
            }
            if (HandleToConstraint.Length < constraintCount)
            {
                //Note that the handle pool, if disposed, will have a -1 highest claimed id, corresponding to a 0 length copy region.
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, constraintCount, handlePool.HighestPossiblyClaimedId + 1);
            }
            //Note that we modify the type batch allocation and pass it to the batches. 
            //The idea here is that the user may also have modified the per-type sizes and wants them to affect the result of this call.
            constraintsPerTypeBatch = Math.Max(1, constraintsPerTypeBatch);
            if (TypeBatchAllocation.MinimumCapacity < constraintsPerTypeBatch)
                TypeBatchAllocation.MinimumCapacity = constraintsPerTypeBatch;
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].EnsureCapacity(TypeBatchAllocation, bodiesCount, TypeCountEstimate);
            }
            //Like the bodies set, we lazily handle handlePool internal capacity unless explicitly told to expand by ensure capacity.
            //This will likely be overkill, but it's a pretty small cost (oh no four hundred kilobytes for a simulation with 100,000 constraints).
            //If the user really wants to stop resizes, well, this will do that.
            handlePool.EnsureCapacity(constraintCount, bufferPool.SpecializeFor<int>());
        }

        public void Compact(int bodiesCount, int constraintCount, int constraintsPerTypeBatch)
        {
            constraintsPerTypeBatch = Math.Max(1, constraintsPerTypeBatch);
            if (TypeBatchAllocation.MinimumCapacity > constraintsPerTypeBatch)
                TypeBatchAllocation.MinimumCapacity = constraintsPerTypeBatch;
            //Note that we cannot safely compact the handles array below the highest potentially allocated id. This could be a little disruptive sometimes, but the cost is low.
            //If it ever ecomes a genuine problem, you can change the way the idpool works to permit a tighter maximum.
            var targetConstraintCount = BufferPool<ConstraintLocation>.GetLowestContainingElementCount(Math.Max(constraintCount, handlePool.HighestPossiblyClaimedId + 1));
            if (HandleToConstraint.Length > targetConstraintCount)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, targetConstraintCount, handlePool.HighestPossiblyClaimedId + 1);
            }
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Compact(TypeBatchAllocation, bodies, bodiesCount);
            }
            handlePool.Compact(constraintCount, bufferPool.SpecializeFor<int>());
        }

        public void Resize(int bodiesCount, int constraintCount, int constraintsPerTypeBatch)
        {
            if (!Batches.Span.Allocated)
            {
                //This solver instance was disposed, so we need to explicitly reconstruct the batches array.
                QuickList<ConstraintBatch, Array<ConstraintBatch>>.Create(new PassthroughArrayPool<ConstraintBatch>(), BatchCountEstimate, out Batches);
            }
            var targetConstraintCount = BufferPool<ConstraintLocation>.GetLowestContainingElementCount(Math.Max(constraintCount, handlePool.HighestPossiblyClaimedId + 1));
            if (HandleToConstraint.Length != targetConstraintCount)
            {
                bufferPool.SpecializeFor<ConstraintLocation>().Resize(ref HandleToConstraint, targetConstraintCount, handlePool.HighestPossiblyClaimedId + 1);
            }
            TypeBatchAllocation.MinimumCapacity = Math.Max(1, constraintsPerTypeBatch);
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Resize(TypeBatchAllocation, bodies, bodiesCount, TypeCountEstimate);
            }
            handlePool.Resize(constraintCount, bufferPool.SpecializeFor<int>());
        }

        /// <summary>
        /// Disposes all resources in the solver, returning unmanaged resources to the pool and dropping all pooled managed resource references.
        /// </summary>
        /// <remarks>The solver object can be reused if EnsureCapacity or Resize is called to rehydrate the resources.</remarks>
        public void Dispose()
        {
            for (int i = 0; i < Batches.Count; ++i)
            {
                Batches[i].Dispose(TypeBatchAllocation);
            }
            bufferPool.SpecializeFor<ConstraintLocation>().Return(ref HandleToConstraint);
            HandleToConstraint = new Buffer<ConstraintLocation>();
            handlePool.Dispose(bufferPool.SpecializeFor<int>());
            Batches.Dispose(new PassthroughArrayPool<ConstraintBatch>());
            Batches = new QuickList<ConstraintBatch, Array<ConstraintBatch>>();
            TypeBatchAllocation.ResetPools();
        }


    }
}
