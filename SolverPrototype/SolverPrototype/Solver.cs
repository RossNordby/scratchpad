using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

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

    public class Solver
    {
        public QuickList<ConstraintBatch> Batches;

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

        IdPool handlePool = new IdPool();
        public ConstraintLocation[] HandlesToConstraints;

        public TypeBatchAllocation TypeBatchAllocation { get; private set; }

        public Solver(Bodies bodies, int iterationCount = 5, int initialCapacity = 1024, int minimumCapacityPerTypeBatch = 64, int initialTypeCountEstimate = 32)
        {
            this.iterationCount = iterationCount;
            this.bodies = bodies;
            Batches = new QuickList<ConstraintBatch>(new PassthroughBufferPool<ConstraintBatch>());
            HandlesToConstraints = new ConstraintLocation[initialCapacity];
            TypeBatchAllocation = new TypeBatchAllocation(initialTypeCountEstimate, minimumCapacityPerTypeBatch);
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
            ref var constraintLocation = ref HandlesToConstraints[handle];
            reference.TypeBatch = Batches.Elements[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId);
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
        public int Allocate(ref int bodyHandles, int bodyCount, int typeId, out ConstraintReference reference)
        {
            int targetBatchIndex = -1;
            //Find the first batch that references none of the bodies that this constraint needs.
            for (int i = 0; i < Batches.Count; ++i)
            {
                var batch = Batches[i];

                bool canFit = true;
                for (int j = 0; j < bodyCount; ++j)
                {
                    if (batch.BodyHandles.Contains(Unsafe.Add(ref bodyHandles, j)))
                    {
                        canFit = false;
                        break;
                    }
                }
                if (canFit)
                {
                    targetBatchIndex = i;
                    break;
                }
            }
            ConstraintBatch targetBatch;
            if (targetBatchIndex == -1)
            {
                //No batch available. Have to create a new one.
                targetBatch = new ConstraintBatch();
                targetBatchIndex = Batches.Count;
                Batches.Add(targetBatch);
            }
            else
            {
                targetBatch = Batches.Elements[targetBatchIndex];
            }
            //Add all the constraint's body handles to the batch we found (or created) to block future references to the same bodies.
            for (int j = 0; j < bodyCount; ++j)
            {
                targetBatch.BodyHandles.Add(Unsafe.Add(ref bodyHandles, j));
            }
            var handle = handlePool.Take();
            targetBatch.Allocate(handle, ref bodyHandles, TypeBatchAllocation, typeId, out reference);

            if (handle >= HandlesToConstraints.Length)
            {
                Array.Resize(ref HandlesToConstraints, HandlesToConstraints.Length << 1);
                Debug.Assert(handle < HandlesToConstraints.Length, "Handle indices should never jump by more than 1 slot, so doubling should always be sufficient.");
            }
            HandlesToConstraints[handle].IndexInTypeBatch = reference.IndexInTypeBatch;
            HandlesToConstraints[handle].TypeId = typeId;
            HandlesToConstraints[handle].BatchIndex = targetBatchIndex;
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
            description.ApplyDescription(constraintReference.TypeBatch, bundleIndex, innerIndex, ref description);
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
            description.ApplyDescription(constraintReference.TypeBatch, bundleIndex, innerIndex, ref description);
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

        /// <summary>
        /// Removes the constraint associated with the given handle. Note that this may invalidate any outstanding direct constraint references (TypeBatch-index pairs)
        /// by reordering the constraints within the TypeBatch subject to removal.
        /// </summary>
        /// <param name="handle">Handle of the constraint to remove from the solver.</param>
        public void Remove(int handle)
        {
            ref var constraintLocation = ref HandlesToConstraints[handle];
            Batches[constraintLocation.BatchIndex].Remove(constraintLocation.TypeId, constraintLocation.IndexInTypeBatch, HandlesToConstraints, TypeBatchAllocation);

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
            ref var location = ref HandlesToConstraints[handle];
            var dummy = default(TConstraintDescription);
            var typeBatch = Batches.Elements[location.BatchIndex].GetTypeBatch(dummy.ConstraintTypeId);
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
            ref var constraintLocation = ref HandlesToConstraints[constraintHandle];
            //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
            //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
            //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
            Batches.Elements[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId).UpdateForBodyMemoryMove(constraintLocation.IndexInTypeBatch, bodyIndexInConstraint, newBodyLocation);
        }

        /// <summary>
        /// Enumerates the set of body indices associated with a constraint in order of their references within the constraint.
        /// </summary>
        /// <param name="constraintHandle">Constraint to enumerate.</param>
        /// <param name="enumerator">Enumerator to use.</param>
        internal void EnumerateConnectedBodyIndices<TEnumerator>(int constraintHandle, ref TEnumerator enumerator) where TEnumerator : IForEach<int>
        {
            ref var constraintLocation = ref HandlesToConstraints[constraintHandle];
            //This does require a virtual call, but memory swaps should not be an ultra-frequent thing.
            //(A few hundred calls per frame in a simulation of 10000 active objects would probably be overkill.)
            //(Also, there's a sufficient number of cache-missy indirections here that a virtual call is pretty irrelevant.)
            Batches.Elements[constraintLocation.BatchIndex].GetTypeBatch(constraintLocation.TypeId).EnumerateConnectedBodyIndices(constraintLocation.IndexInTypeBatch, ref enumerator);
        }

        //TODO: Note that removals are a little tricky. In order to reduce the number of batches which persist, every removal
        //should attempt to fill the entity reference gap left by the removal with a constraint from another batch if possible.
        //Unfortunately, this requires a way to look up the constraint that is holding a given reference. That makes the
        //bitfield approach insufficient- have to store indices...
        //An alternative is to keep remove as simple and direct as possible, possibly leaving more batches than there would ideally be.
        //Then, later, a dedicated batch analysis process tries to find any potential compressions.
        //The dedicated batch analysis has some pretty nice advantages:
        //0) Removes stay (relatively) fast- no O(n) searching or complex logic.
        //1) High churn adds/removes are extremely common during chaotic collisions, which is exactly when you need as little overhead as possible. 
        //1.5) High churn situations will tend to rapidly invalidate the 'optimization' effort of extremely aggressive on-remove swaps.
        //2) On-removal will often fail to make any change due to other reference blockages.
        //3) Dedicated batch analysis can be deferred over multiple frames because the intermediate results are all fine from a correctness standpoint. 
        //3.5) Deferred costs can be kept consistently low no matter what kind of add/remove churn is happening.
        //4) Dedicated batch analysis can be performed asynchronously and hidden behind other stally stages which aren't the solver and don't modify the solver (e.g. broadphase, midphase).
        //5) Even if we are in a 'suboptimal' constraint configuration (i.e. some pulldowns exist), it will rarely have an effect on performance unless it actually results in extra batches.
        //6) Dedicated analysis could afford to perform more complex heuristics to optimize batches.

        public void Update(float dt, float inverseDt)
        {
            //TODO: Note that the prestep phase is completely parallel. When multithreading, there is no need to stop at the boundaries of bodybatches.
            //You could technically build a separate list that ignores bodybatch boundaries. Value is unclear.
            for (int i = 0; i < Batches.Count; ++i)
            {
                var batch = Batches.Elements[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches.Elements[j].Prestep(bodies.LocalInertiaBundles, dt, inverseDt);
                }
            }
            for (int i = 0; i < Batches.Count; ++i)
            {
                var batch = Batches.Elements[i];
                for (int j = 0; j < batch.TypeBatches.Count; ++j)
                {
                    batch.TypeBatches.Elements[j].WarmStart(bodies.VelocityBundles);
                }
            }
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                for (int i = 0; i < Batches.Count; ++i)
                {
                    var batch = Batches.Elements[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        batch.TypeBatches.Elements[j].SolveIteration(bodies.VelocityBundles);
                    }
                }
            }
        }


    }
}
