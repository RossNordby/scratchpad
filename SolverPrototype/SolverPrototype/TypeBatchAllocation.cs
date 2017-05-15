using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;

namespace SolverPrototype
{
    public class TypeBatchAllocation
    {
        int minimumCapacity;
        /// <summary>
        /// Gets or sets the minimum amount of space, in constraints, initially allocated in a type batch.
        /// </summary>
        public int MinimumCapacity
        {
            get { return minimumCapacity; }
            set
            {
                if (value <= 0)
                    throw new ArgumentException("Minimum capacity must be positive.");
                minimumCapacity = value;
            }
        }

        /// <summary>
        /// Pool used to initialize type batch buffers.
        /// </summary>
        public BufferPool BufferPool { get; private set; }

        int[] capacities;
        Pool<TypeBatch>[] pools;

        public TypeBatchAllocation(int initialTypeCountEstimate, int minimumCapacity, BufferPool bufferPool)
        {
            capacities = new int[initialTypeCountEstimate];
            pools = new Pool<TypeBatch>[initialTypeCountEstimate];
            this.minimumCapacity = minimumCapacity;
            this.BufferPool = bufferPool;
        }

        //TODO: There is really no good reason why you couldn't have all the types defined up front. It would avoid all the last-second resizing. Not exactly a big issue,
        //but something to think about if this ever gets to be any more cumbersome.
        void Validate(int typeId)
        {
            Debug.Assert(typeId >= 0, "Type ids are nonnegative!");
            if (typeId >= capacities.Length)
            {
                var newSize = 1 << SpanHelper.GetContainingPowerOf2(typeId);
                Array.Resize(ref capacities, newSize);
                Array.Resize(ref pools, newSize);
            }
        }

        /// <summary>
        /// Gets or sets the capacity associated with a given type id.
        /// </summary>
        /// <param name="typeId">Id of the type to get or set the initial capacity for.</param>
        /// <returns>The initial capacity for the given type.</returns>
        public int this[int typeId]
        {
            get
            {
                Validate(typeId);
                return Math.Max(capacities[typeId], minimumCapacity);
            }
            set
            {
                Validate(typeId);
                capacities[typeId] = value;

            }
        }

        /// <summary>
        /// Sets the initial capacity for the given type id to be the highest of the given value and the current stored value.
        /// </summary>
        /// <param name="typeId">Type id to change the capacity for.</param>
        public void EnsureTypeCapacity(int typeId, int capacity)
        {
            Validate(typeId);
            capacities[typeId] = Math.Max(capacity, capacities[typeId]);
        }

        /// <summary>
        /// Resets all type capacities to zero. Leaves the minimum capacity across all constraints unchanged.
        /// </summary>
        public void ResetPerTypeCapacities()
        {
            Array.Clear(capacities, 0, capacities.Length);
        }

        //Note that neither of these pooling functions are thread safe. The assumption is that there is too much shared mutable state flying around
        //in the systems where these would be called for per-system fine grained locking to have any value.
        /// <summary>
        /// Requests an initialized TypeBatch of the specified type. Not thread safe.
        /// </summary>
        /// <returns>TypeBatch of the specified type.</returns>
        public TypeBatch Take(int typeId)
        {
            Validate(typeId);
            if (pools[typeId] == null)
            {
                //The new constraint results in constructors being invoked with reflection at the moment, but we shouldn't be creating new type batches frequently.
                //If you've got 32 constraint types and 128 batches, you'll need a total of 4096 invocations. It's a very small cost.               
                pools[typeId] = new Pool<TypeBatch>(() => (TypeBatch)Activator.CreateInstance(ConstraintTypeIds.GetType(typeId)), cleaner: batch => batch.Reset(BufferPool));
            }
            var typeBatch = pools[typeId].Take();
            //We didn't initialize it in the pool; do so here. (The pool COULD use an initializer, but, well, I didn't do that.)
            //TODO: In the future, the initialization would ideally take a memory source to pull from rather than just using a static BufferPool source.
            //This TypeBatchAllocation class would have a reference to that memory source so we could very conveniently pass it into the initialization.
            typeBatch.Initialize(BufferPool, BundleIndexing.GetBundleCount(Math.Max(minimumCapacity, capacities[typeId])), typeId);
            return typeBatch;
        }

        /// <summary>
        /// Returns a type batch to the pool. Not thread safe.
        /// </summary>
        /// <param name="typeId">Type id of the TypeBatch to return.</param>
        /// <param name="typeBatch">Batch to return.</param>
        public void Return(TypeBatch typeBatch, int typeId)
        {
            Validate(typeId);
            Debug.Assert(pools[typeId] != null, "Can't return something to a pool that doesn't exist.");
            Debug.Assert(typeBatch.GetType() == ConstraintTypeIds.GetType(typeId), "Type of returned type batch and the type implied by the type id must match.");
            pools[typeId].Return(typeBatch);
        }

        /// <summary>
        /// Resets all TypeBatch pools. Does not affect any TypeBatches requested from the pool that are still outstanding.
        /// </summary>
        public void ResetPools()
        {
            for (int i = 0; i < pools.Length; ++i)
            {
                pools[i]?.Clear();
            }
        }
    }
}
