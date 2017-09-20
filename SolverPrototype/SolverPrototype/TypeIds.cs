using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using SolverPrototype.CollisionDetection;
using SolverPrototype.CollisionDetection.CollisionTasks;

namespace SolverPrototype
{
    /// <summary>
    /// Helper class to register the default types of constraints and shapes with the engine.
    /// </summary>
    public static class DefaultTypes
    {
        /// <summary>
        /// Registers the set of shapes constraints that are packaged in the engine.
        /// </summary>
        public static void Register()
        {
            TypeIds<IShape>.Register<Sphere>();
            TypeIds<TypeBatch>.Register<BallSocketTypeBatch>();
            TypeIds<TypeBatch>.Register<ContactManifold1TypeBatch>();
            TypeIds<TypeBatch>.Register<ContactManifold4TypeBatch>();
        }

        public static CollisionTaskRegistry CreateDefaultCollisionTaskRegistry()
        {
            var collisionTaskRegistry = new CollisionTaskRegistry();
            collisionTaskRegistry.Register(new SpherePairCollisionTask());
            return collisionTaskRegistry;
        }
    }

    /// <summary>
    /// Handles the registration and retrieval of type ids for constraint batch types.
    /// </summary>
    /// <typeparam name="TTypeParent">Parent type of types registered in this type id set.</typeparam>
    /// <remarks>
    /// Nothing in this class is thread safe. It is assumed that calls to Register, Clear, and GetId are always safely synchronized.
    /// </remarks>
    public static class TypeIds<TTypeParent>
    {
        static class Ids<T>
        {
            internal static int Id;
        }

        static List<Type> registeredTypes = new List<Type>();


        /// <summary>
        /// Gets the number of types that have been registered.
        /// </summary>
        public static int RegisteredTypeCount { get { return registeredTypes.Count; } }

        [Conditional("DEBUG")]
        static void ValidateType<T>()
        {
            Debug.Assert(registeredTypes.Contains(typeof(T)), "Type must exist in the registered type set.");
        }
        /// <summary>
        /// Gets the id associated with the given type.
        /// </summary>
        /// <typeparam name="T">Type to look up the id of.</typeparam>
        /// <returns>Id of the given type.</returns>
        /// <remarks>Not thread safe with calls to Reset, Register, or ChangeMinimumCapacity. All changes to registration should be be performed outside of any usage.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int GetId<T>() where T : TTypeParent
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
            Debug.Assert(typeId >= 0 && typeId < registeredTypes.Count);
            return registeredTypes[typeId];
        }

        /// <summary>
        /// Gets the id associated with a given type. This is a slow path. Use the generic path whenever possible.
        /// </summary>
        /// <param name="type">Type to look up the index of.</param>
        /// <returns>Id of the given type if it was registered. -1 otherwise.</returns>
        public static int GetId(Type type)
        {
            return registeredTypes.IndexOf(type);
        }
        
        /// <summary>
        /// Registers a type in the id set. If the type was already registered, the existing id is returned.
        /// </summary>
        /// <typeparam name="T">Type to register.</typeparam>
        /// <returns>Id associated with the type.</returns>
        public static int Register<T>() where T : TTypeParent, new()
        {
            var newType = typeof(T);
            var index = registeredTypes.IndexOf(newType);
            if (index > -1)
            {
                Debug.Assert(Ids<T>.Id == index);
                return index;
            }
            index = registeredTypes.Count;
            registeredTypes.Add(newType);
            Ids<T>.Id = index;
            return index;
        }
        
        /// <summary>
        /// Clears all type id registrations.
        /// </summary>
        public static void Clear()
        {
            registeredTypes.Clear();
        }

    }
}
