using BEPUutilities2.ResourceManagement;
using System;
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
    public abstract class PrestepTypeBatch
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
        public abstract void Add<T>(ref T constraint, SolveBatchSet solveBatchSet);
        public abstract void Remove(int constraintHandle);

        public abstract void Prestep();

        /// <summary>
        /// Relinquishes resources to pools and returns to a freshly created state.
        /// </summary>
        public abstract void Reset();
        
    }


    public static class ConstraintTypeRegistration
    {
        static class DescriptionIds<T> where T : IConstraintDescription
        {
            internal static int Id;
        }
        static class BatchIds<T> where T : PrestepTypeBatch
        {
            internal static int Id;
        }
        static class DescriptionIds<T> where T : IConstraintDescription
        {
            internal static int Id;
        }

        static HashSet<Type> registeredBatchTypes = new HashSet<Type>();


        public void GetBatchId<T>() where T : PrestepTypeBatch
        {
            Debug.Assert(registeredTypes.Contains(typeof(T)));
        }

        public void Register<TDescription, TBatch>()
        {

        }
    }
}
