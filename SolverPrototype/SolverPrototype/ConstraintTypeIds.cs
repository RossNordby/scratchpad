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

        static List<Type> registeredBatchTypes = new List<Type>();

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
        /// Gets the type associated with a given type id.
        /// </summary>
        /// <param name="typeId">Type id to check the type of.</param>
        /// <returns>Type associated with the given type id.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Type GetType(int typeId)
        {
            Debug.Assert(typeId >= 0 && typeId < registeredBatchTypes.Count);
            return registeredBatchTypes[typeId];
        }

        /// <summary>
        /// Gets the id associated with a given type. This is a slow path. Use the generic path whenever possible.
        /// </summary>
        /// <param name="type">Type to look up the index of.</param>
        /// <returns>Id of the given type if it was registered. -1 otherwise.</returns>
        public static int GetId(Type type)
        {
            return registeredBatchTypes.IndexOf(type);
        }
        
        /// <summary>
        /// Registers a type in the id set. If the type was already registered, the existing id is returned.
        /// </summary>
        /// <typeparam name="T">Type to register.</typeparam>
        /// <returns>Id associated with the type.</returns>
        public static int Register<T>() where T : TypeBatch, new()
        {
            var newType = typeof(T);
            var index = registeredBatchTypes.IndexOf(newType);
            if (index > -1)
            {
                Debug.Assert(Ids<T>.Id == index);
                return index;
            }
            index = registeredBatchTypes.Count;
            registeredBatchTypes.Add(newType);
            Ids<T>.Id = index;
            return index;
        }
        
        /// <summary>
        /// Clears all type id registrations.
        /// </summary>
        public static void Clear()
        {
            registeredBatchTypes.Clear();
        }

    }
}
