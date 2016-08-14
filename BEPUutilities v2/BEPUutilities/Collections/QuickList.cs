using System;
using System.Diagnostics;
using BEPUutilities2.ResourceManagement;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace BEPUutilities2.Collections
{
    /// <summary>
    /// Container supporting list-like behaviors built on top of pooled arrays.
    /// </summary>
    /// <remarks>Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care, it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, it rarely checks input for errors, and the enumerator doesn't check for mid-enumeration modification.</remarks>
    /// <typeparam name="T">Type of the elements in the list.</typeparam>
    public struct QuickList<T> : IDisposable, IList<T>
    {
        private int poolIndex;
        /// <summary>
        /// Gets the buffer pool index associated with the elements array.
        /// </summary>
        public int PoolIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return poolIndex;
            }
        }

        private BufferPool<T> pool;
        /// <summary>
        /// Pool from which element arrays are pulled.
        /// </summary>
        public BufferPool<T> Pool
        {
            get
            {
                return pool;
            }
        }

        /// <summary>
        /// Gets the backing array containing the elements of the list.
        /// Indices from 0 to Count-1 hold actual data. All other data is undefined.
        /// </summary>
        public readonly T[] Elements;

        private int count;
        /// <summary>
        /// Gets or sets the number of elements in the list.
        /// </summary>
        public int Count
        {
            get { return count; }
            set
            {
                Debug.Assert(value >= 0 && value <= Elements.Length, "Count should fit in the current backing array length.");
                count = value;
            }
        }


        /// <summary>
        /// Gets an element at the given index in the list.
        /// </summary>
        /// <param name="index">Index to grab an element from.</param>
        /// <returns>Element at the given index in the list.</returns>
        public T this[int index]
        {
            //You would think that such a trivial accessor would inline without any external suggestion.
            //Sometimes, yes. Sometimes, no. :(
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ValidateIndex(index);
                return Elements[index];
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                ValidateIndex(index);
                Elements[index] = value;
            }
        }

        /// <summary>
        /// Creates a new list.
        /// </summary>
        /// <param name="pool">Pool from which to retrieve typed arrays.</param>
        /// <param name="initialPoolIndex">Initial pool index to pull the backing array from. The size of the initial buffer will be 2^initialPoolIndex.</param>
        public QuickList(BufferPool<T> pool, int initialPoolIndex = 5)
        {
            this.pool = pool;
            poolIndex = initialPoolIndex;
            Elements = pool.TakeFromPoolIndex(poolIndex);

            count = 0;

        }

        /// <summary>
        /// Ensures that the list has enough room to hold the specified number of elements.
        /// </summary>
        /// <param name="count">Number of elements to hold.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnsureCapacity(int count)
        {
            if (count > Elements.Length)
            {
                Resize(BufferPool.GetPoolIndex(count));
            }
        }

        private void Resize(int newPoolIndex)
        {
            Debug.Assert(Count <= (1 << newPoolIndex), "New pool index must contain all elements in the list.");
            var oldList = this;
            this = new QuickList<T>(pool, newPoolIndex);
            Count = oldList.Count;
            Array.Copy(oldList.Elements, Elements, Count);

            //The array may contain reference types.
            //While the user can opt into leaking references if they really want to, it shouldn't be unavoidable.
            //Clear it before disposal to avoid leaking references.
            //(TODO: This clear could be narrowed to arrays of managed types.)
            if (!typeof(T).IsPrimitive)
            {
                oldList.Clear();
            }
            oldList.Dispose();
        }

        /// <summary>
        /// Adds the elements of a list to the QuickList.
        /// </summary>
        /// <param name="list">List to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddRange(IList<T> list)
        {
            var newCount = count + list.Count;
            EnsureCapacity(newCount);
            list.CopyTo(Elements, count);
            count = newCount;
        }

        /// <summary>
        /// Adds the elements of a list to the QuickList.
        /// </summary>
        /// <param name="list">List to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddRange(ref QuickList<T> list)
        {
            var newCount = count + list.Count;
            EnsureCapacity(newCount);
            list.CopyTo(Elements, count);
            count = newCount;
        }

        /// <summary>
        /// Adds the element to the list.
        /// </summary>
        /// <param name="element">Item to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(T element)
        {
            Validate();
            if (Count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Adds the element to the list without checking the count against the capacity.
        /// </summary>
        /// <param name="element">Item to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddUnsafely(T element)
        {
            Validate();
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Adds the element to the list.
        /// </summary>
        /// <param name="element">Element to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(ref T element)
        {
            Validate();
            if (Count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Adds the element to the list without checking the count against the capacity.
        /// </summary>
        /// <param name="element">Element to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddUnsafely(ref T element)
        {
            Validate();
            Elements[Count] = element;
            ++Count;
        }

        /// <summary>
        /// Gets the index of the element in the list, if present.
        /// </summary>
        /// <param name="element">Element to find.</param>
        /// <returns>Index of the element in the list if present, -1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element)
        {
            Validate();
            return Array.IndexOf(Elements, element, 0, Count);
        }

        /// <summary>
        /// Gets the index of the element in the list, if present.
        /// </summary>
        /// <param name="element">Element to find.</param>
        /// <returns>Index of the element in the list if present, -1 otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref T element)
        {
            Validate();
            return Array.IndexOf(Elements, element, 0, Count);
        }

        /// <summary>
        /// Removes an element from the list. Preserves the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Remove(T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                RemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list. Preserves the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Remove(ref T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                RemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list. Does not preserve the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool FastRemove(T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                FastRemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list. Does not preserve the order of elements.
        /// </summary>
        /// <param name="element">Element to remove from the list.</param>
        /// <returns>True if the element was present and was removed, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool FastRemove(ref T element)
        {
            Validate();
            var index = IndexOf(element);
            if (index >= 0)
            {
                FastRemoveAt(index);
                return true;
            }
            return false;
        }

        /// <summary>
        /// Removes an element from the list at the given index. Preserves the order of elements.
        /// </summary>
        /// <param name="index">Index of the element to remove from the list.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveAt(int index)
        {
            Validate();
            ValidateIndex(index);
            --Count;
            if (index < Count)
            {
                //Copy everything from the removal point onward backward one slot.
                Array.Copy(Elements, index + 1, Elements, index, Count - index);
            }
            //Clear out the former last slot.
            Elements[Count] = default(T);
        }

        /// <summary>
        /// Removes an element from the list at the given index. Does not preserve the order of elements.
        /// </summary>
        /// <param name="index">Index of the element to remove from the list.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void FastRemoveAt(int index)
        {
            Validate();
            ValidateIndex(index);
            --Count;
            if (index < Count)
            {
                //Put the final element in the removed slot.
                Elements[index] = Elements[Count];
            }
            //Clear out the former last slot.
            Elements[Count] = default(T);
        }


        /// <summary>
        /// Removes and outputs the last element in the list if it exists.
        /// </summary>
        /// <param name="element">Last element of the list.</param>
        /// <returns>True if the element existed and was removed, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryPop(out T element)
        {
            Validate();
            if (Count > 0)
            {
                Count--;
                element = Elements[Count];
                return true;
            }
            element = default(T);
            return false;
        }

        /// <summary>
        /// Inserts an item to the <see cref="T:System.Collections.Generic.IList`1"/> at the specified index.
        /// </summary>
        /// <param name="index">The zero-based index at which <paramref name="element"/> should be inserted.</param>
        /// <param name="element">The object to insert into the <see cref="T:System.Collections.Generic.IList`1"/>.</param>
        /// <exception cref="T:System.ArgumentOutOfRangeException"><paramref name="index"/> is not a valid index in the <see cref="T:System.Collections.Generic.IList`1"/>.</exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Insert(int index, T element)
        {
            Validate();
            if (Count == Elements.Length)
                Resize(poolIndex + 1);
            Array.Copy(Elements, index, Elements, index + 1, count - index);
            Elements[index] = element;
            ++Count;
        }

        /// <summary>
        /// Inserts an item to the <see cref="T:System.Collections.Generic.IList`1"/> at the specified index.
        /// </summary>
        /// <param name="index">The zero-based index at which <paramref name="element"/> should be inserted.</param>
        /// <param name="element">The object to insert into the <see cref="T:System.Collections.Generic.IList`1"/>.</param>
        /// <exception cref="T:System.ArgumentOutOfRangeException"><paramref name="index"/> is not a valid index in the <see cref="T:System.Collections.Generic.IList`1"/>.</exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Insert(int index, ref T element)
        {
            Validate();
            if (Count == Elements.Length)
                Resize(poolIndex + 1);
            Array.Copy(Elements, index, Elements, index + 1, count - index);
            Elements[index] = element;
            ++Count;
        }


        /// <summary>
        /// Determines whether the <see cref="T:System.Collections.Generic.ICollection`1"/> contains a specific value.
        /// </summary>
        /// <returns>
        /// true if <paramref name="element"/> is found in the <see cref="T:System.Collections.Generic.ICollection`1"/>; otherwise, false.
        /// </returns>
        /// <param name="element">The object to locate in the <see cref="T:System.Collections.Generic.ICollection`1"/>.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(T element)
        {
            return IndexOf(element) >= 0;
        }

        /// <summary>
        /// Determines whether the <see cref="T:System.Collections.Generic.ICollection`1"/> contains a specific value.
        /// </summary>
        /// <returns>
        /// true if <paramref name="element"/> is found in the <see cref="T:System.Collections.Generic.ICollection`1"/>; otherwise, false.
        /// </returns>
        /// <param name="element">The object to locate in the <see cref="T:System.Collections.Generic.ICollection`1"/>.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(ref T element)
        {
            return IndexOf(ref element) >= 0;
        }

        /// <summary>
        /// Copies the elements of the <see cref="T:System.Collections.Generic.ICollection`1"/> to an <see cref="T:System.Array"/>, starting at a particular <see cref="T:System.Array"/> index.
        /// </summary>
        /// <param name="array">The one-dimensional <see cref="T:System.Array"/> that is the destination of the elements copied from <see cref="T:System.Collections.Generic.ICollection`1"/>. The <see cref="T:System.Array"/> must have zero-based indexing.</param><param name="arrayIndex">The zero-based index in <paramref name="array"/> at which copying begins.</param><exception cref="T:System.ArgumentNullException"><paramref name="array"/> is null.</exception><exception cref="T:System.ArgumentOutOfRangeException"><paramref name="arrayIndex"/> is less than 0.</exception><exception cref="T:System.ArgumentException">The number of elements in the source <see cref="T:System.Collections.Generic.ICollection`1"/> is greater than the available space from <paramref name="arrayIndex"/> to the end of the destination <paramref name="array"/>.</exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo(T[] array, int arrayIndex)
        {
            Array.Copy(Elements, 0, array, arrayIndex, Count);
        }


        /// <summary>
        /// Clears the list by setting the count to zero and explicitly setting all relevant indices in the backing array to default values.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            Validate();
            Array.Clear(Elements, 0, Count);
            Count = 0;
        }

        /// <summary>
        /// Compacts the internal buffer to the minimum size required for the number of elements in the list.
        /// </summary>
        public void Compact()
        {
            Validate();
            var newPoolIndex = BufferPool.GetPoolIndex(Count);
            if (newPoolIndex != poolIndex)
                Resize(newPoolIndex);
        }

        /// <summary>
        /// Returns the list's buffer. Does not clear the buffer.
        /// </summary>
        public void Dispose()
        {
            pool.Return(Elements, poolIndex);
#if DEBUG
            pool = null;
#endif
        }

        [Conditional("DEBUG")]
        void ValidateIndex(int index)
        {
            Debug.Assert(index >= 0 && index < Count, "Index must be nonnegative and less than the number of elements in the list.");
        }


        [Conditional("DEBUG")]
        private void Validate()
        {
            Debug.Assert(pool != null, "Should not use a default-constructed or disposed QuickList.");
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(Elements, Count);
        }

        IEnumerator<T> IEnumerable<T>.GetEnumerator()
        {
            return GetEnumerator();
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
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



        /// <summary>
        /// Gets a value indicating whether the <see cref="T:System.Collections.Generic.ICollection`1"/> is read-only.
        /// </summary>
        /// <returns>
        /// true if the <see cref="T:System.Collections.Generic.ICollection`1"/> is read-only; otherwise, false.
        /// </returns>
        bool ICollection<T>.IsReadOnly
        {
            get { return false; }
        }
    }
}
