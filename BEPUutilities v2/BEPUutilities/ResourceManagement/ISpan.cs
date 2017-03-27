using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilities2.ResourceManagement
{
    /// <summary>
    /// Defines a type that can act as an indexable typed wrapper for a block of memory.
    /// </summary>
    /// <remarks>
    /// The motivation here is allow tons of pool-based collections in an array without requiring GC inspection for underlying pool references.
    /// The primary use case for the ISpan{T} is heap-stored pinned pointers, but this layer of abstraction is needed for Quick* collections to also work for managed types.
    /// This is distinct from the .NET Span{T} type, which is a stack-only construct that has to be 'rehydrated' from another type.
    /// Ideally, you could use a true Span{T} wrapped in a supporting ISpan{T} struct with all of the quick collections too,
    /// but the stack-only property (and lack of a generalized language-level stack-only concept) currently disallows this. Maybe later!
    /// </remarks>
    /// <typeparam name="T">Type of the elements in the memory block.</typeparam>
    public interface ISpan<T>
    {
        //Note that this type does not require slicing support. While it could be supported by the implementations of the interface, we don't require it.

        /// <summary>
        /// Gets a reference to the element at the given index.
        /// </summary>
        /// <param name="index">Index of the element to get the reference of.</param>
        /// <returns>Reference of the element at the given index.</returns>
        ref T this[int index] { get; }

        /// <summary>
        /// Gets the number of elements in the span.
        /// </summary>
        int Length { get; }


        /// <summary>
        /// Copies elements from one span region to another.
        /// </summary>
        /// <remarks>
        /// If the span regions overlap, the result is the same as if the original values of the source were preserved in a temporary location before writing to the target.
        /// </remarks>
        /// <typeparam name="TTargetSpan">The type of the span to copy into.</typeparam>
        /// <param name="sourceStart">Start index of the copy source region.</param>
        /// <param name="targetSpan">Span to copy into.</param>
        /// <param name="targetStart">Start index of the copy target region.</param>
        /// <param name="count">Number of elements to copy.</param>
        void CopyTo<TTargetSpan>(int sourceStart, ref TTargetSpan targetSpan, int targetStart, int count) where TTargetSpan : ISpan<T>;

        /// <summary>
        /// Clears the memory in a subset of the span.
        /// </summary>
        /// <param name="start">Inclusive starting index of the clear operation.</param>
        /// <param name="count">Number of indices to clear.</param>
        void Clear(int start, int count);

        /// <summary>
        /// Clears any references included in the span. Memory that isn't a managed reference may or may not be left untouched.
        /// </summary>
        /// <remarks>
        /// In many cases, we can use a pointer-backed span or an otherwise pure valuetype span that cannot include managed references. In those cases,
        /// this does absolutely nothing- which is good!
        /// </remarks>
        /// <param name="start">Inclusive starting index of the clear operation.</param>
        /// <param name="count">Number of indices to clear.</param>
        void ClearManagedReferences(int start, int count);

        /// <summary>
        /// Gets the index of the first occurrence of an element in the span as determined by the default comparison. Search proceeds from low to high indices.
        /// </summary>
        /// <param name="element">Element to search for.</param>
        /// <param name="start">Inclusive start index of the search.</param>
        /// <param name="count">Number of elements to search.</param>
        /// <returns>Index of the element in the span if it is present; -1 otherwise.</returns>
        int IndexOf(T element, int start, int count);

        /// <summary>
        /// Gets the index of the first occurrence of an element in the span as determined by the default comparison. Search proceeds from low to high indices.
        /// </summary>
        /// <param name="element">Element to search for.</param>
        /// <param name="start">Inclusive start index of the search.</param>
        /// <param name="count">Number of elements to search.</param>
        /// <returns>Index of the element in the span if it is present; -1 otherwise.</returns>
        int IndexOf(ref T element, int start, int count);

        /// <summary>
        /// Gets the index of the first match of a predicate. Search proceeds from low to high indices.
        /// </summary>
        /// <param name="start">Inclusive start index of the search.</param>
        /// <param name="count">Number of elements to search.</param>
        /// <param name="predicate">Predicate to test against the elements in the span region.</param>
        /// <returns>Index of the element in the span if it is present; -1 otherwise.</returns>
        int IndexOf<TPredicate>(int start, int count, ref TPredicate predicate) where TPredicate : IPredicate<T>;

    }

    /// <summary>
    /// Defines a type able to match an element.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public interface IPredicate<T>
    {
        //We're assuming here that the inlining will be good enough that we won't pay extra for passing by ref under any circumstance. This isn't always the case.
        bool Matches(ref T item);
    }
    /// <summary>
    /// Defines a type capable of performing the hashing and equality comparisons necessary for hash based collections.
    /// </summary>
    /// <typeparam name="T">Type of the elements to be hashed and compared.</typeparam>
    public interface IEqualityComparerRef<T>
    {
        int GetHashCode(ref T item);
        bool Equals(ref T a, ref T b);
    }

    /// <summary>
    /// IEqualityComparerRef wrapper around an EqualityComparer.
    /// </summary>
    /// <typeparam name="T">Type of the objects to compare and hash.</typeparam>
    public struct WrapperEqualityComparer<T> : IEqualityComparerRef<T>
    {
        public EqualityComparer<T> Comparer;
        /// <summary>
        /// Creates a default comparer for the given type.
        /// </summary>
        /// <param name="item">Item to compare against other items.</param>
        /// <param name="predicate">Predicate to test against other items using the default comparer for this type.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateDefault(ref T item, out WrapperEqualityComparer<T> predicate)
        {
            predicate.Comparer = EqualityComparer<T>.Default;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref T a, ref T b)
        {
            return Comparer.Equals(a, b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetHashCode(ref T item)
        {
            return item.GetHashCode();
        }
    }

    /// <summary>
    /// IPredicate wrapper around an EqualityComparer and an object to compare against.
    /// </summary>
    /// <typeparam name="T">Type of the objects to compare.</typeparam>
    public struct WrapperPredicate<T> : IPredicate<T>
    {
        public T Item;
        public EqualityComparer<T> Comparer;
        /// <summary>
        /// Creates a default comparer for the given type.
        /// </summary>
        /// <param name="item">Item to compare against other items.</param>
        /// <param name="predicate">Predicate to test against other items using the default comparer for this type.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CreateDefault(ref T item, out WrapperPredicate<T> predicate)
        {
            predicate.Item = item;
            predicate.Comparer = EqualityComparer<T>.Default;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Matches(ref T otherItem)
        {
            return Comparer.Equals(Item, otherItem);
        }
    }
    
}
