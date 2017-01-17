using BEPUutilities2.ResourceManagement;
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

            internal static Pool<T> Pool;
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
        /// <remarks>Not thread safe with calls to Reset or Register. All changes to registration should be be performed outside of any usage.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetId<T>() where T : TypeBatch
        {
            ValidateType<T>();
            return Ids<T>.Id;
        }

        /// <summary>
        /// Gets a batch of the specified type.
        /// </summary>
        /// <typeparam name="T">Type of the batch to grab.</typeparam>
        /// <returns>Batch of the specified type.</returns>
        public static T Take<T>()
        {
            ValidateType<T>();
            return Ids<T>.Pool.LockingTake();
        }
        /// <summary>
        /// Returns a batch to its pool.
        /// </summary>
        /// <typeparam name="T">Type of the batch to return.</typeparam>
        /// <param name="batch">Batch to return.</param>
        public static void Return<T>(T batch)
        {
            ValidateType<T>();
            Ids<T>.Pool.LockingReturn(batch);
        }
        /// <summary>
        /// Clears all type id registrations.
        /// </summary>
        public static void Clear()
        {
            //Gross? Gross. But better than leaving unreachable reference types floating around forever.
            foreach (var type in registeredBatchTypes)
            {
                Type.GetType("Ids`1").MakeGenericType(type).GetField("Pool").SetValue(null, null);
            }
            registeredBatchTypes.Clear();
        }

        /// <summary>
        /// Registers a type in the id set.
        /// </summary>
        /// <typeparam name="T">Type to register.</typeparam>
        /// <returns>Id associated with the type.</returns>
        public static int Register<T>() where T : TypeBatch, new()
        {
            var index = registeredBatchTypes.Count;
            if (!registeredBatchTypes.Add(typeof(T)))
            {
                throw new ArgumentException("Type is already registered; cannot reregister.");
            }
            Ids<T>.Id = index;
            //The new constraint results in constructors being invoked with reflection at the moment, but we shouldn't be creating new type batches frequently.
            //If you've got 32 constraint types and 128 batches, you'll need a total of 4096 invocations. It's a very small cost.
            Ids<T>.Pool = new Pool<T>(() => new T(), batch => batch.Initialize(), batch => batch.Reset());
            return index;
        }
    }
}
