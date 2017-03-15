using BEPUutilities2.ResourceManagement;
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
        int[] capacities;

        public TypeBatchAllocation(int initialTypeCountEstimate, int minimumCapacity)
        {
            capacities = new int[initialTypeCountEstimate];
            this.minimumCapacity = minimumCapacity;
        }

        void Validate(int typeId)
        {
            Debug.Assert(typeId >= 0, "Type ids are nonnegative!");
            if (typeId >= capacities.Length)
                Array.Resize(ref capacities, 1 << BufferPool.GetPoolIndex(typeId));
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
    }
}
