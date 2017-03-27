using System;
using System.Collections.Generic;
using System.Diagnostics;
using BEPUutilities2.ResourceManagement;
using System.Runtime.CompilerServices;

namespace BEPUutilities2.Collections
{
    /// <summary>
    /// Container supporting constant time adds and removes while preserving fast iteration times.
    /// Offers very direct access to information at the cost of safety.
    /// </summary>
    /// <remarks><para>Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care, it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, it is particularly vulnerable to bad hash functions, 
    /// it rarely checks input for errors, and the enumerator doesn't check for mid-enumeration modification.</para>
    /// <para>Note that the implementation is extremely simple. It uses single-step linear probing under the assumption of very low collision rates.
    /// A generous table capacity is recommended; this trades some memory for simplicity and runtime performance.</para></remarks>
    /// <typeparam name="T">Type of element held by the container.</typeparam>
    public struct QuickSet<T, TSpan, TTableSpan, TEqualityComparer>
        where TSpan : ISpan<T>
        where TTableSpan : ISpan<int>
        where TEqualityComparer : IEqualityComparerRef<T>
    {
        /// <summary>
        /// Gets the number of elements in the set.
        /// </summary>
        public int Count;
        /// <summary>
        /// Mask for use in performing fast modulo operations for hashes. Requires that the table span is a power of 2.
        /// </summary>
        public int TableMask;

        /// <summary>
        /// Backing memory of the set's table. Values are distributed according to the EqualityComparer's hash function.
        /// Slots containing 0 are unused and point to nothing. Slots containing higher values are equal to one plus the index of an element in the Span.
        /// </summary>
        public TTableSpan Table;

        /// <summary>
        /// Backing memory containing the elements of the set.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public TSpan Span;

        /// <summary>
        /// Equality comparer used 
        /// </summary>
        public TEqualityComparer EqualityComparer;


        /// <summary>
        /// Gets or sets an element at the given index in the list representation.
        /// </summary>
        /// <param name="index">Index to grab an element from.</param>
        /// <returns>Element at the given index in the list.</returns>
        public ref T this[int index]
        {
            //You would think that such a trivial accessor would inline without any external suggestion.
            //Sometimes, yes. Sometimes, no. :(
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Debug.Assert(index >= 0 && index < Count, "Index should be within the set's size.");
                return ref Span[index];
            }
        }

        /// <summary>
        /// Creates a new set.
        /// </summary>
        /// <param name="initialSpan">Span to use as backing memory of the set elements.</param>
        /// <param name="initialTableSpan">Span to use as backing memory of the table.</param>
        /// <param name="comparer">Comparer to use for the set.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickSet(ref TSpan initialSpan, ref TTableSpan initialTableSpan, TEqualityComparer comparer)
        {
            ValidateSpanCapacity(ref initialSpan, ref initialTableSpan);
            Span = initialSpan;
            Table = initialTableSpan;
            TableMask = Table.Length - 1;
            Count = 0;
            EqualityComparer = comparer;
            Debug.Assert(EqualityComparer != null);
        }

        /// <summary>
        /// Creates a new set using a default constructed equality comparer.
        /// </summary>
        /// <param name="initialSpan">Span to use as backing memory of the set elements.</param>
        /// <param name="initialTableSpan">Span to use as backing memory of the table.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickSet(ref TSpan initialSpan, ref TTableSpan initialTableSpan)
            : this(ref initialSpan, ref initialTableSpan, default(TEqualityComparer))
        {
        }

        /// <summary>
        /// Creates a new set.
        /// </summary>
        /// <param name="pool">Pool to pull a span from.</param>   
        /// <param name="tablePool">Pool to pull a table span from.</param>
        /// <param name="initialElementPoolIndex">Initial pool index to pull the object buffer from. The size of the initial buffer will be 2^initialElementPoolIndex.</param>
        /// <param name="tableSizePower">Initial pool index to pull the object buffer from. The size of the initial table buffer will be 2^(initialElementPoolIndex + tableSizePower).</param>
        /// <param name="comparer">Comparer to use in the set.</param>
        /// <param name="set">Created set.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create<TPool, TTablePool>(TPool pool, TTablePool tablePool, int initialElementPoolIndex, int tableSizePower, TEqualityComparer comparer,
            out QuickSet<T, TSpan, TTableSpan, TEqualityComparer> set)
            where TPool : IMemoryPool<T, TSpan>
            where TTablePool : IMemoryPool<int, TTableSpan>
        {
            pool.TakeForPower(initialElementPoolIndex, out var span);
            tablePool.TakeForPower(initialElementPoolIndex + tableSizePower, out var tableSpan);
            set = new QuickSet<T, TSpan, TTableSpan, TEqualityComparer>(ref span, ref tableSpan, comparer);
        }
        /// <summary>
        /// Creates a new set with a default constructed comparer.
        /// </summary>
        /// <param name="pool">Pool to pull a span from.</param>   
        /// <param name="tablePool">Pool to pull a table span from.</param>
        /// <param name="initialElementPoolIndex">Initial pool index to pull the object buffer from. The size of the initial buffer will be 2^initialElementPoolIndex.</param>
        /// <param name="tableSizePower">Initial pool index to pull the object buffer from. The size of the initial table buffer will be 2^(initialElementPoolIndex + tableSizePower).</param>
        /// <param name="set">Created set.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create<TPool, TTablePool>(TPool pool, TTablePool tablePool, int initialElementPoolIndex, int tableSizePower,
            out QuickSet<T, TSpan, TTableSpan, TEqualityComparer> set)
            where TPool : IMemoryPool<T, TSpan>
            where TTablePool : IMemoryPool<int, TTableSpan>
        {
            Create(pool, tablePool, initialElementPoolIndex, tableSizePower, default(TEqualityComparer), out set);
        }

        /// <summary>
        /// Swaps out the set's backing memory span for a new span.
        /// If the new span is smaller, the set's count is truncated and the extra elements are dropped. 
        /// The old span is not cleared or returned to any pool; if it needs to be pooled or cleared, the user must handle it.
        /// </summary>
        /// <param name="newSpan">New span to use.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize(ref TSpan newSpan, ref TTableSpan newTableSpan, out TSpan oldSpan, out TTableSpan oldTableSpan)
        {
            ValidateSpanCapacity(ref newSpan, ref newTableSpan);
            var oldSet = this;
            Span = newSpan;
            Table = newTableSpan;
            Count = 0;
            TableMask = newTableSpan.Length - 1;
            var newCount = oldSet.Count > newSpan.Length ? newSpan.Length : oldSet.Count;

            //Unfortunately we can't really do a straight copy; the backing table relies on modulo operations.
            //Technically, we could copy the regular set and then rely on a partial add to take care of the rest, but bleh!
            //Should really attempt to avoid resizes on sets and dictionaries whenever possible anyway. It ain't fast.
            for (int i = 0; i < newCount; ++i)
            {
                Add(ref oldSet.Span[i]);
            }

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Resize(int newObjectPoolIndex, int newTablePoolIndex)
        {
            Debug.Assert(Count <= (1 << newObjectPoolIndex), "New pool index must contain all elements.");
            //Just double the size of the set.
            var oldSet = this;
            this = new QuickSet<T>(ElementPool, TablePool, newObjectPoolIndex, newTablePoolIndex - newObjectPoolIndex);
            for (int i = 0; i < oldSet.Count; ++i)
            {
                Add(oldSet.Span[i]);
            }

            //The elements array may contain reference types.
            //While the user can opt into leaking references if they really want to, it shouldn't be unavoidable.
            //Clear it before disposal to avoid leaking references.
            //Note that only the elements array is cleared; the table array contains only primitives and does not need to be cleared.
            //(TODO: This clear could be narrowed to arrays of managed types.)
            if (!typeof(T).IsPrimitive)
                Array.Clear(oldSet.Span, 0, oldSet.Count);
            oldSet.Dispose();
        }

        /// <summary>
        /// Ensures that the set has enough room to hold the specified number of elements.
        /// </summary>
        /// <param name="count">Number of elements to hold.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnsureCapacity(int count)
        {
            if (count > Span.Length)
            {
                var newPoolIndex = BufferPool.GetPoolIndex(count);
                Resize(newPoolIndex, tablePoolIndex - elementPoolIndex + newPoolIndex);
            }
        }


        /// <summary>
        /// Gets the index of the element in the table.
        /// </summary>
        /// <param name="element">Element to look up.</param>
        /// <param name="tableIndex">Index of the element in the redirect table, or if it is not present, the index of where it would be added.</param>
        /// <param name="elementIndex">The index of the element in the elements array, if it exists; -1 otherwise.</param>
        /// <returns>True if the element is present in the set, false if it is not.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool GetTableIndices(T element, out int tableIndex, out int elementIndex)
        {
            Validate();
            //The table lengths are guaranteed to be a power of 2, so the modulo is a simple binary operation.
            tableIndex = QuickDictionary<int, int>.Rehash(element.GetHashCode()) & TableMask;
            //0 in the table means 'not taken'; all other values are offset by 1 upward. That is, 1 is actually index 0, 2 is actually index 1, and so on.
            //This is preferred over using a negative number for flagging since clean buffers will contain all 0's.
            while ((elementIndex = Table[tableIndex]) > 0)
            {
                //This table index is taken. Is this the specified element?
                //Remember to decode the object index.
                if (Span[--elementIndex].Equals(element))
                {
                    return true;
                }
                tableIndex = (tableIndex + 1) & TableMask;
            }
            elementIndex = -1;
            return false;
        }

        /// <summary>
        /// Gets the index of the element in the list if it exists.
        /// </summary>
        /// <param name="element">Element to get the index of.</param>
        /// <returns>The index of the element if the element exists in the list, -1 otherwise.</returns>
        public int IndexOf(T element)
        {
            Validate();
            int tableIndex, objectIndex;
            GetTableIndices(element, out tableIndex, out objectIndex);
            return objectIndex;
        }

        /// <summary>
        /// Checks if a given element already belongs to the set.
        /// </summary>
        /// <param name="element">Element to test for.</param>
        /// <returns>True if the element already belongs to the set, false otherwise.</returns>
        public bool Contains(T element)
        {
            Validate();
            int tableIndex, objectIndex;
            return GetTableIndices(element, out tableIndex, out objectIndex);
        }

        /// <summary>
        /// Adds an element to the set. If a version of the element (same hash code, 'equal' by comparer) is already present,
        /// it is replaced by the given version.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present and was instead replaced.</returns>
        public bool AddAndReplace(T element)
        {
            Validate();
            if (Count == Span.Length)
            {
                //There's no room left; resize.
                Resize(elementPoolIndex + 1, tablePoolIndex + 1);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }

            int tableIndex, elementIndex;
            if (GetTableIndices(element, out tableIndex, out elementIndex))
            {
                //Already present!
                Span[elementIndex] = element;
                return false;
            }


            //It wasn't in the set. Add it!
            Span[Count] = element;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = ++Count;
            return true;
        }


        /// <summary>
        /// Adds an element to the set if it is not already present.
        /// </summary>
        /// <param name="element">Element to add.</param>
        /// <returns>True if the element was added to the set, false if the element was already present.</returns>
        public bool Add(T element)
        {
            Validate();

            if (Count == Span.Length)
            {
                //There's no room left; resize.
                Resize(elementPoolIndex + 1, tablePoolIndex + 1);

                //Note that this is tested before any indices are found.
                //If we resized only after determining that it was going to be added,
                //the potential resize would invalidate the computed indices.
            }

            int tableIndex, elementIndex;
            if (GetTableIndices(element, out tableIndex, out elementIndex))
            {
                //Already present!
                return false;
            }


            //It wasn't in the set. Add it!
            Span[Count] = element;
            //Use the encoding- all indices are offset by 1 since 0 represents 'empty'.
            Table[tableIndex] = ++Count;
            return true;
        }

        //Note: the reason this is named "FastRemove" instead of just "Remove" despite it being the only remove present is that
        //there may later exist an order preserving "Remove". That would be a very sneaky breaking change.

        /// <summary>
        /// Removes an element from the set if belongs to the set.
        /// Does not preserve the order of elements in the set.
        /// </summary>
        /// <param name="element">Element to remove.</param>
        /// <returns>True if the element was found and removed, false otherwise.</returns>
        public bool FastRemove(T element)
        {
            Validate();
            //Find it.
            int tableIndex, objectIndex;
            if (GetTableIndices(element, out tableIndex, out objectIndex))
            {
                //We found the object!
                //Add and remove must both maintain a property:
                //All items are either at their desired index (as defined by the hash), or they are contained in a contiguous block clockwise from the desired index.
                //Removals seek to fill the gap they create by searching clockwise to find items which can be moved backward.
                //Search clockwise for an item to fill this slot. The search must continue until a gap is found.
                int moveCandidateIndex;
                int gapIndex = tableIndex;
                //Search clockwise.
                while ((moveCandidateIndex = Table[tableIndex = (tableIndex + 1) & TableMask]) > 0)
                {
                    //This slot contains something. What is its actual index?
                    --moveCandidateIndex;
                    int desiredIndex = QuickDictionary<int, int>.Rehash(Span[moveCandidateIndex].GetHashCode()) & TableMask;

                    //Would this element be closer to its actual index if it was moved to the gap?
                    //To find out, compute the clockwise distance from the gap and the clockwise distance from the ideal location.

                    var distanceFromGap = (tableIndex - gapIndex) & TableMask;
                    var distanceFromIdeal = (tableIndex - desiredIndex) & TableMask;
                    if (distanceFromGap <= distanceFromIdeal)
                    {
                        //The distance to the gap is less than or equal the distance to the ideal location, so just move to the gap.
                        Table[gapIndex] = Table[tableIndex];
                        gapIndex = tableIndex;
                    }

                }
                //Clear the table gap left by the removal.
                Table[gapIndex] = 0;
                //Swap the final element into the removed object's element array index, if the removed object wasn't the last object.
                --Count;
                if (objectIndex < Count)
                {
                    Span[objectIndex] = Span[Count];
                    //Locate the swapped object in the table and update its index.
                    int oldObjectIndex;
                    GetTableIndices(Span[objectIndex], out tableIndex, out oldObjectIndex);
                    Table[tableIndex] = objectIndex + 1; //Remember the encoding! all indices offset by 1.
                }
                //Clear the final slot in the elements set.
                Span[Count] = default(T);

                return true;
            }
            return false;
        }


        /// <summary>
        /// Shrinks the internal buffers to the smallest acceptable size and releases the old buffers to the pools.
        /// </summary>
        public void Compact()
        {
            Validate();
            var minimumRequiredPoolIndex = BufferPool.GetPoolIndex(Count);
            if (minimumRequiredPoolIndex != elementPoolIndex)
                Resize(minimumRequiredPoolIndex, minimumRequiredPoolIndex + (tablePoolIndex - elementPoolIndex));
        }

        /// <summary>
        /// Removes all elements from the set.
        /// </summary>
        public void Clear()
        {
            //While it may be appealing to remove individual elements from the set when the set is sparse,
            //using a brute force clear over the entire table is almost always faster. And it's a lot simpler!
            Table.Clear(0, Table.Length);
            Span.Clear(0, Count);
            Count = 0;
        }

        /// <summary>
        /// Removes all elements from the set without modifying the contents of the elements array. Be careful about using this with reference types; it may leak references into the pool.
        /// </summary>
        public void FastClear()
        {
            Table.Clear(0, Table.Length);
            Count = 0;
        }

        public Enumerator GetEnumerator()
        {
            Validate();
            return new Enumerator(Span, Count);
        }


        public struct Enumerator : IEnumerator<T>
        {
            private readonly T[] backingArray;
            private readonly int count;
            private int index;

            public Enumerator(T[] backingArray, int count)
            {
                this.backingArray = backingArray;
                this.count = count;

                index = -1;
            }

            public T Current
            {
                get { return backingArray[index]; }
            }

            public void Dispose()
            {
            }

            object System.Collections.IEnumerator.Current
            {
                get { return Current; }
            }

            public bool MoveNext()
            {
                return ++index < count;
            }

            public void Reset()
            {
                index = -1;
            }
        }
        [Conditional("DEBUG")]
        void ValidateSpanCapacity(ref TSpan span, ref TTableSpan tableSpan)
        {
            Debug.Assert(tableSpan.Length >= span.Length, "The table span must be at least as large as the element span.");
            Debug.Assert((tableSpan.Length & (tableSpan.Length - 1)) == 0, "Set depend upon power of 2 backing table span sizes for efficient modulo operations.");
        }

        [Conditional("DEBUG")]
        private void Validate()
        {
            ValidateSpanCapacity(ref Span, ref Table);
            Debug.Assert(Table.Length != 0, "The QuickSet must have its internal buffers and pools available; default-constructed or disposed QuickSets should not be used.");
        }





    }
}
