using BEPUutilities2.ResourceManagement;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Handles the registration and retrieval of type ids for constraint batch types.
    /// </summary>
    /// <remarks>
    /// Nothing in this class is thread safe. It is assumed that calls to Register, Clear, and GetId are always safely synchronized.
    /// </remarks>
    public static class ConstraintTypeIds
    {
        static class Ids<T>
        {
            internal static int Id;
        }


        static HashSet<Type> registeredBatchTypes = new HashSet<Type>();

        [Conditional("DEBUG")]
        static void ValidateType<T>()
        {
            Debug.Assert(registeredBatchTypes.Contains(typeof(T)), "Type must exist in the constraint type set.");
        }
        /// <summary>
        /// Gets the id associated with the given type.
        /// </summary>
        /// <typeparam name="T">Type to look up the id of.</typeparam>
        /// <returns>Id of the given type.</returns>
        /// <remarks>Not thread safe with calls to Reset, Register, or ChangeMinimumCapacity. All changes to registration should be be performed outside of any usage.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetId<T>() where T : TypeBatch
        {
            ValidateType<T>();
            return Ids<T>.Id;
        }
        
        /// <summary>
        /// Clears all type id registrations.
        /// </summary>
        public static void Clear()
        {
            registeredBatchTypes.Clear();
        }

        /// <summary>
        /// Registers a type in the id set.
        /// </summary>
        /// <typeparam name="T">Type to register.</typeparam>
        /// When a type batch is requested, its capacity will be the larger of this value and the requested capacity.</param>
        /// <returns>Id associated with the type.</returns>
        public static int Register<T>() where T : TypeBatch, new()
        {
            var index = registeredBatchTypes.Count;
            if (!registeredBatchTypes.Add(typeof(T)))
            {
                throw new ArgumentException("Type is already registered; cannot reregister.");
            }
            Ids<T>.Id = index;
            return index;
        }
        
    }
}
