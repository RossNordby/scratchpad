//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Runtime.CompilerServices;

//namespace SolverPrototype
//{
//    public interface IComparerRef<T>
//    {
//        int Compare(ref T a, ref T b);
//    }

//    /// <summary>
//    /// Requires a type expose equality and hashcode functions with ref parameters. Ref equivalent of IEqualityComparer.
//    /// </summary>
//    /// <typeparam name="T">Type to compare.</typeparam>
//    public interface IEqualityComparerRef<T>
//    {
//        bool Equals(ref T x, ref T y);
//        int GetHashCode(ref T i);
//    }
//    public interface IPredicate<T>
//    {
//        bool Test(T item);
//    }
//    public interface IPredicateRef<T>
//    {
//        bool Test(ref T item);
//    }

//    //Note that this design is probably not going to stick around in this form. While the idea of a referenceless storage representation is necessary, the 'hydrated' version
//    //of this list is terribly inconvenient to use. Which is to say, there really isn't a hydrated representation, just operations on the dehydrated version.
//    //Also, the introduction of a runtime-aware Span<T> type will likely change this quite a bit.
//    //(The only issue there is that Span<T> is stack only, last checked.
//    //And I'm not aware of any generalization of that stack only property, which means any OOPish representation gets torpedoed.)
//    //(In the worst case we can always just resort to pinned pointers. The Unsafe functionality makes it easy to do what we need from there efficiently. 
//    //Only downside there is that we're fighting the GC. We could always just allocate a big block of unmanaged memory if such functionality is exposed in a cross platform manner...)
//    public struct SuballocatedList
//    {
//        public BufferRegion Region;
//        public int Count;

//        //Note that we don't technically constrain the type to be a struct. It's not even a sufficient condition for safety, since structs can contain references.
//        //Rather than cluttering everything with a bunch of security theater, trust the user.
//        //Who knows, maybe they really want to put a managed reference type in an array such that the GC can no longer track it!

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void Add<T>(SuballocatedBufferPool bufferPool, ref SuballocatedList list, ref T item)
//        {
//            if (list.Count == list.Region.GetLengthForType<T>())
//            {
//                bufferPool.Resize(ref list.Region, list.Region.Power + 1);
//            }
//            Unsafe.Add(ref bufferPool.GetStart<T>(ref list.Region), list.Count) = item;
//            ++list.Count;
//        }
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void Add<T>(SuballocatedBufferPool bufferPool, ref SuballocatedList list, T item)
//        {
//            //TODO: Confirm internal inlining.
//            Add(bufferPool, ref list, ref item);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void Create(SuballocatedBufferPool bufferPool, int initialPower, out SuballocatedList suballocatedList)
//        {
//            bufferPool.Allocate(initialPower, out suballocatedList.Region);
//            suballocatedList.Count = 0;
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void Dispose(SuballocatedBufferPool bufferPool, ref SuballocatedList suballocatedList)
//        {
//            bufferPool.Free(ref suballocatedList.Region);
//        }

//        //Keeping with the quick-collection convention of 'fast' removes meaning that the last element is pulled into the removed slot, rather than preserving order.
//        //Note that we're using an extremely non-idiomatic comparison scheme here- we require that a type parameter be provided that specifies a struct
//        //implementor of the comparison interface. You could reasonably ask why:
//        //1) IEquatable doesn't give you the option to specify different forms of equatability, and can run into annoyances when an external type doesn't support it.
//        //2) Providing a comparison delegate incurs delegate call overhead.
//        //3) Using a struct comparer forces JIT specialization.
//        //This is basically working around a language expressiveness limitation.
//        //A little bit questionable, but this is in large part just a test to see how it works out.

//        //Also note that none of the removal functions bother to clear values to defaults.
//        //That means pointers to reference types could remain in the array- but don't worry everything is fine, because the GC can't see those references anyway.
//        //The backing array is byte[]. :) :) :         )

//        /// <summary>
//        /// Removes the first item from the list that satisfies the given predicate.
//        /// </summary>
//        /// <typeparam name="T">Type of the item to remove.</typeparam>
//        /// <typeparam name="TPredicate">Predicate used to determine whether to remove an item.</typeparam>
//        /// <param name="bufferPool">Buffer pool that the list was allocated from.</param>
//        /// <param name="list">List to remove from.</param>
//        /// <param name="item">Item to remove from the list.</param>
//        /// <returns>True if the item was present and removed, false otherwise.</returns>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static bool FastRemove<T, TPredicate>(SuballocatedBufferPool bufferPool, ref SuballocatedList list, ref TPredicate predicate) where TPredicate : struct, IPredicate<T>
//        {
//            ref var start = ref bufferPool.GetStart<T>(ref list.Region);
//            for (int i = 0; i < list.Count; ++i)
//            {
//                if (predicate.Test(Unsafe.Add(ref start, i)))
//                {
//                    FastRemoveAt<T>(bufferPool, ref list, i);
//                    return true;
//                }
//            }
//            return false;
//        }



//        /// <summary>
//        /// Removes an element at the given index.
//        /// </summary>
//        /// <typeparam name="T">Type of the removed element.</typeparam>
//        /// <param name="bufferPool">Buffer pool that the list was allocated from.</param>
//        /// <param name="list">List to remove from.</param>
//        /// <param name="index">Index to remove in terms of elements of the specified type.</param>
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static void FastRemoveAt<T>(SuballocatedBufferPool bufferPool, ref SuballocatedList list, int index)
//        {
//            Debug.Assert(index >= 0 && index < list.Count);
//            --list.Count;
//            if (index != list.Count)
//            {
//                //Have to swap the last element into the removed slot.
//                ref var start = ref bufferPool.GetStart<T>(ref list.Region);
//                Unsafe.Add(ref start, index) = Unsafe.Add(ref start, list.Count);
//            }
//        }


//        //This is just a little workaround for the difficulty associated with examining the values. 
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        public static List<T> DebugView<T>(ref SuballocatedList list, SuballocatedBufferPool pool)
//        {
//            var debugList = new List<T>(list.Count);
//            ref var start = ref pool.GetStart<T>(ref list.Region);
//            for (int i = 0; i < list.Count; ++i)
//            {
//                debugList.Add(Unsafe.Add(ref start, i));
//            }
//            return debugList;
//        }

//    }



//}
