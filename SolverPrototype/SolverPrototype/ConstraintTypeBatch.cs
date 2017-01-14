using BEPUutilities2.ResourceManagement;
using System.Collections.Generic;
using System.Diagnostics;

namespace SolverPrototype
{
    /// <summary>
    /// Superclass of the per-type batches that exist within each unique-bodies batch.
    /// </summary>
    /// <remarks>
    /// While we bite the bullet on some additional complexity when adding and removing from these batches due to the different types involved,
    /// splitting up constraints by type is extremely important for performance. It allows the use of wide SIMD processing to span multiple constraint instances and
    /// permits the cache friendly contiguous storage of constraint properties.
    /// (And it avoids performing virtual dispatches at every constraint solve, but that's not very important.)
    /// </remarks>
    public abstract class ConstraintTypeBatch
    {
        /// <summary>
        /// Gets the constraint type index associated with this type batch.
        /// </summary>
        public abstract int ConstraintTypeIndex { get; }
        /// <summary>
        /// Gets the number of constraints in the type batch.
        /// </summary>
        public abstract int ConstraintCount { get; }

        //Note that the type parameter is at the level of a function.
        //It is technically possible to misuse this and add with an invalid type.
        //This would require an internal bug of some kind, though- something like an invalid constraint type id.
        //We'll catch that failure with runtime debug asserts.
        public abstract void Add<T>(ref T constraint);
        public abstract void Remove(int constraintHandle);

        public abstract void Prestep();
        public abstract void WarmStart();
        public abstract void SolveIteration();

        /// <summary>
        /// Relinquishes resources to pools and returns to a freshly created state.
        /// </summary>
        public abstract void Reset();

        /// <summary>
        /// List of type batch pools, indices matching up with the constraint type produced by the pool.
        /// If you want to add to it, use the RegisterConstraintType function.
        /// </summary>
        /// <remarks>Modifying this through anything but the RegisterConstraintType function can break a lot of stuff. Be careful about what you do and when you do it.</remarks>
        public readonly static List<Pool<ConstraintTypeBatch>> TypeBatchPools = new List<Pool<ConstraintTypeBatch>>();

        /// <summary>
        /// Claims an index for a constraint type.
        /// Not safe to call while adding constraints to a constraint batch or while the engine is updating.
        /// </summary>
        /// <returns>Registered index.</returns>
        public static int RegisterConstraintType(Pool<ConstraintTypeBatch> typeBatchPool)
        {
            lock (TypeBatchPools)
            {
                Debug.Assert(!TypeBatchPools.Contains(typeBatchPool), "Can't register the same type pool multiple times!");
                var typeIndex = TypeBatchPools.Count;
                TypeBatchPools.Add(typeBatchPool);
                return typeIndex;
            }
        }

    }
}
