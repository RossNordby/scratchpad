using System;
using System.Diagnostics;
using BEPUutilities2.ResourceManagement;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace BEPUutilities2.Collections
{
    /// <summary>
    /// Container supporting double ended queue behaviors built on top of pooled arrays.
    /// </summary>
    /// <remarks>
    /// Be very careful when using this type. It has sacrificed a lot upon the altar of performance; a few notable issues include:
    /// it is a value type and copying it around will break things without extreme care,
    /// it cannot be validly default-constructed,
    /// it exposes internal structures to user modification, 
    /// it rarely checks input for errors,
    /// the enumerator doesn't check for mid-enumeration modification,
    /// it allows unsafe addition that can break if the user doesn't manage the capacity,
    /// it works on top of an abstracted memory blob which might internally be a pointer that could be rugpulled, 
    /// it does not (and is incapable of) checking that provided memory gets returned to the same pool that it came from.
    /// </remarks>
    /// <typeparam name="T">Type of the elements in the queue.</typeparam>
    /// <typeparam name="TSpan">Type of the memory span backing the list.</typeparam>
    public struct QuickQueue<T, TSpan> where TSpan : ISpan<T>
    {
        /// <summary>
        /// Number of elements in the queue.
        /// </summary>
        public int Count;

        /// <summary>
        /// Index of the first element in the queue.
        /// </summary>
        public int FirstIndex;

        /// <summary>
        /// Index of the last element in the queue.
        /// </summary>
        public int LastIndex;

        /// <summary>
        /// Mask based on the current span length used to do fast modulo operations; requires that the span has a power of 2 length.
        /// </summary>
        public int CapacityMask;

        /// <summary>
        /// Gets the backing memory containing the elements of the queue.
        /// Indices from FirstIndex to LastIndex inclusive hold actual data. All other data is undefined.
        /// Watch out for wrap around; LastIndex can be less than FirstIndex even when count > 0!
        /// </summary>
        public TSpan Span;



        /// <summary>
        /// Gets the backing array index for the logical queue index.
        /// </summary>
        /// <param name="queueIndex">Index in the logical queue.</param>
        /// <returns>The index in in the backing array corresponding to the given logical queue index.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetBackingArrayIndex(int queueIndex)
        {
            return (FirstIndex + queueIndex) & CapacityMask;
        }

        /// <summary>
        /// Gets an element at the given index in the queue.
        /// 0 gets the element at the FirstIndex. Count-1 would get the element at LastIndex.
        /// </summary>
        /// <param name="index">Index to grab an element from.</param>
        /// <returns>Element at the given index in the queue.</returns>
        public ref T this[int index]
        {
            //TODO: Check inlining.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ValidateIndex(index);
                return ref Span[GetBackingArrayIndex(index)];
            }
        }


        /// <summary>
        /// Creates a new queue.
        /// </summary>
        /// <param name="initialSpan">Span to use as backing memory to begin with.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public QuickQueue(ref TSpan initialSpan)
        {
            Span = initialSpan;
            Count = 0;
            CapacityMask = Span.Length - 1;
            FirstIndex = 0;
            LastIndex = CapacityMask;
        }
        /// <summary>
        /// Creates a new queue.
        /// </summary>
        /// <param name="pool">Pool to pull a span from.</param>
        /// <param name="minimumInitialCount">The minimum size of the region to be pulled from the pool. Actual span may be larger.</param>
        /// <param name="queue">Created queue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create<TPool>(TPool pool, int minimumInitialCount, out QuickQueue<T, TSpan> queue) where TPool : IMemoryPool<T, TSpan>
        {
            pool.Take(minimumInitialCount, out queue.Span);
            queue.Count = 0;
            queue.CapacityMask = queue.Span.Length - 1;
            queue.FirstIndex = 0;
            queue.LastIndex = queue.CapacityMask;

        }


        /// <summary>
        /// Swaps out the queue's backing memory span for a new span.
        /// If the new span is smaller, the queue's count is truncated and the extra elements are dropped. 
        /// The old span is not cleared or returned to any pool; if it needs to be pooled or cleared, the user must handle it.
        /// </summary>
        /// <param name="newSpan">New span to use.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize(ref TSpan newSpan, out TSpan oldSpan)
        {
            Validate();
            var oldQueue = this;
            //Truncate length.
            if (oldQueue.Count > newSpan.Length)
                Count = newSpan.Length;
            else
                Count = oldQueue.Count;
            LastIndex = Count - 1;
            FirstIndex = 0;
            CapacityMask = newSpan.Length - 1;
            Span = newSpan;

            oldSpan = oldQueue.Span;

            //There is no guarantee that the count is equal to Elements.Length, so both cases must be covered.
            if (oldQueue.LastIndex >= oldQueue.FirstIndex)
            {
                //The indices are in order and the list has at least one element in it, so just do one contiguous copy.
                oldSpan.CopyTo(oldQueue.FirstIndex, ref Span, 0, Count);
            }
            else if (Count > 0)
            {
                //The last index is before the first index, meaning the elements wrap around the end of the span. Do a copy for each contiguous region.
                var firstToEndLength = oldSpan.Length - oldQueue.FirstIndex;
                oldSpan.CopyTo(oldQueue.FirstIndex, ref Span, 0, firstToEndLength);
                oldSpan.CopyTo(0, ref Span, firstToEndLength, oldQueue.LastIndex + 1);
            }

        }

        /// <summary>
        /// Resizes the list's backing array for the given size as a power of two.
        /// </summary>
        /// <typeparam name="TPool">Type of the span pool.</typeparam>
        /// <param name="newSizePower">Exponent of the size of the new memory block. New size will be 2^newSizePower.</param>
        /// <param name="pool">Pool to pull a new span from and return the old span to.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize<TPool>(int newSizePower, TPool pool) where TPool : IMemoryPool<T, TSpan>
        {
            Validate();
            Debug.Assert(Count <= (1 << newSizePower), "New pool index must contain all elements.");
            var oldSpan = Span;
            pool.TakeForPower(newSizePower, out )
            var oldQueue = this;
            this = new QuickQueue<T, TSpan>(pool, newSizePower);
            count = oldQueue.Count;


            FirstIndex = 0;
            LastIndex = count - 1;

            //The array may contain reference types.
            //While the user can opt into leaking references if they really want to, it shouldn't be unavoidable.
            //Clear it before disposal to avoid leaking references.
            //(TODO: This clear could be narrowed to arrays of managed types.)
            if (!typeof(T).IsPrimitive)
            {
                oldQueue.Clear();
            }
            oldQueue.Dispose();
        }


        /// <summary>
        /// Ensures that the queue has enough room to hold the specified number of elements.
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

        /// <summary>
        /// Enqueues the element to the end of the queue, incrementing the last index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Enqueue(T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(LastIndex = ((LastIndex + 1) & CapacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Enqueues the element to the start of the queue, decrementing the first index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnqueueFirst(T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(FirstIndex = ((FirstIndex - 1) & CapacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Enqueues the element to the end of the queue, incrementing the last index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Enqueue(ref T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(LastIndex = ((LastIndex + 1) & CapacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Enqueues the element to the start of the queue, decrementing the first index.
        /// </summary>
        /// <param name="element">Item to enqueue.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void EnqueueFirst(ref T element)
        {
            Validate();
            if (count == Elements.Length)
                Resize(poolIndex + 1);
            Elements[(FirstIndex = ((FirstIndex - 1) & CapacityMask))] = element;
            ++count;
        }

        /// <summary>
        /// Dequeues an element from the start of the queue, incrementing the first index.
        /// </summary>
        /// <returns>Element removed from the queue.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T Dequeue()
        {
            Validate();
            if (count == 0)
                throw new InvalidOperationException("The queue is empty.");
            var element = Elements[FirstIndex];
            DeleteFirst();
            return element;

        }

        /// <summary>
        /// Dequeues an element from the end of the queue, decrementing the last index.
        /// </summary>
        /// <returns>Element removed from the queue.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T DequeueLast()
        {
            Validate();
            if (count == 0)
                throw new InvalidOperationException("The queue is empty.");
            var element = Elements[LastIndex];
            DeleteLast();
            return element;

        }

        /// <summary>
        /// Attempts to dequeue an element from the start of the queue, incrementing the first index.
        /// </summary>
        /// <param name="element">Element removed from the queue, if any.</param>
        /// <returns>True if an element was available to remove, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryDequeue(out T element)
        {
            Validate();
            if (count > 0)
            {
                element = Elements[FirstIndex];
                DeleteFirst();
                return true;
            }
            element = default(T);
            return false;

        }

        /// <summary>
        /// Attempts to dequeue an element from the end of the queue, decrementing the last index.
        /// </summary>
        /// <param name="element">Element removed from the queue, if any.</param>
        /// <returns>True if an element was available to remove, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryDequeueLast(out T element)
        {
            Validate();
            if (count > 0)
            {
                element = Elements[LastIndex];
                DeleteLast();
                return true;
            }
            element = default(T);
            return false;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void DeleteFirst()
        {
            Elements[FirstIndex] = default(T);
            FirstIndex = (FirstIndex + 1) & CapacityMask;
            --count;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void DeleteLast()
        {
            Elements[LastIndex] = default(T);
            LastIndex = (LastIndex - 1) & CapacityMask;
            --count;
        }

        /// <summary>
        /// Removes the element at the given index, preserving the order of the queue.
        /// </summary>
        /// <param name="queueIndex">Index in the queue to remove. The index is in terms of the conceptual queue, not the backing array.</param>
        public void RemoveAt(int queueIndex)
        {
            Validate();
            ValidateIndex(queueIndex);
            var arrayIndex = GetBackingArrayIndex(queueIndex);
            if (LastIndex == arrayIndex)
            {
                DeleteLast();
                return;
            }
            if (FirstIndex == arrayIndex)
            {
                DeleteFirst();
                return;
            }
            //It's internal.

            //Four possible cases:
            //1) Queue wraps around end and arrayIndex is in [0, lastIndex),
            //2) Queue wraps around end and arrayIndex is in (firstIndex, arrayLength),
            //3) Queue is contiguous and arrayIndex is closer to lastIndex than firstIndex, or
            //4) Queue is contiguous and arrayIndex is closer to firstIndex than lastIndex
            //In cases #1 and #3, we should move [arrayIndex + 1, lastIndex] to [arrayIndex, lastIndex - 1], deleting the last element.
            //In cases #2 and #4, we should move [firstIndex, arrayIndex - 1] to [firstIndex + 1, arrayIndex], deleting the first element.

            if ((FirstIndex > LastIndex && arrayIndex < LastIndex) || //Case 1
                (FirstIndex < LastIndex && (LastIndex - arrayIndex) < (arrayIndex - FirstIndex))) //Case 3
            {
                Array.Copy(Elements, arrayIndex + 1, Elements, arrayIndex, LastIndex - arrayIndex);
                DeleteLast();
            }
            else
            {
                Array.Copy(Elements, FirstIndex, Elements, FirstIndex + 1, arrayIndex - FirstIndex);
                DeleteFirst();
            }
        }

        /// <summary>
        /// Copies the elements of the <see cref="T:System.Collections.Generic.ICollection`1"/> to an <see cref="T:System.Array"/>, starting at a particular <see cref="T:System.Array"/> index.
        /// </summary>
        /// <param name="array">The one-dimensional <see cref="T:System.Array"/> that is the destination of the elements copied from <see cref="T:System.Collections.Generic.ICollection`1"/>. The <see cref="T:System.Array"/> must have zero-based indexing.</param><param name="arrayIndex">The zero-based index in <paramref name="array"/> at which copying begins.</param><exception cref="T:System.ArgumentNullException"><paramref name="array"/> is null.</exception><exception cref="T:System.ArgumentOutOfRangeException"><paramref name="arrayIndex"/> is less than 0.</exception><exception cref="T:System.ArgumentException">The number of elements in the source <see cref="T:System.Collections.Generic.ICollection`1"/> is greater than the available space from <paramref name="arrayIndex"/> to the end of the destination <paramref name="array"/>.</exception>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo(T[] array, int arrayIndex)
        {
            if (count > 0)
            {
                if (FirstIndex <= LastIndex)
                {
                    Array.Copy(Elements, FirstIndex, array, arrayIndex, count);
                }
                else
                {
                    //Copy the old first-end to the first part of the new array.
                    Array.Copy(Elements, FirstIndex, array, arrayIndex, Elements.Length - FirstIndex);
                    //Copy the old begin-last to the second part of the new array.
                    Array.Copy(Elements, 0, array, arrayIndex + Elements.Length - FirstIndex, LastIndex + 1);
                }
            }
        }

        /// <summary>
        /// Clears the queue by setting the count to zero and explicitly setting all relevant indices in the backing array to default values.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            Validate();
            if (LastIndex >= FirstIndex)
            {
                Array.Clear(Elements, FirstIndex, count);
            }
            else if (count > 0)
            {
                Array.Clear(Elements, FirstIndex, Elements.Length - FirstIndex);
                Array.Clear(Elements, 0, LastIndex + 1);
            }
            count = 0;
            FirstIndex = 0;
            LastIndex = CapacityMask; //length - 1
        }

        /// <summary>
        /// Clears the queue without changing any of the values in the backing array. Be careful about using this if the queue contains reference types.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void FastClear()
        {
            count = 0;
            FirstIndex = 0;
            LastIndex = CapacityMask;
        }

        /// <summary>
        /// Compacts the internal buffer to the minimum size required for the number of elements in the queue.
        /// </summary>
        public void Compact()
        {
            Validate();
            var newPoolIndex = BufferPool.GetPoolIndex(Count);
            if (newPoolIndex != poolIndex)
                Resize(newPoolIndex);
        }

        [Conditional("DEBUG")]
        void ValidateIndex(int index)
        {
            Debug.Assert(index >= 0 && index < Count, "Index must be nonnegative and less than the number of elements in the queue.");
        }

        [Conditional("DEBUG")]
        void ValidateSpanCapacity(ref TSpan span)
        {
            Debug.Assert((span.Length & (span.Length - 1)) == 0, "Queues depend upon power of 2 backing span sizes for efficient modulo operations.");
        }

        [Conditional("DEBUG")]
        private void Validate()
        {
            ValidateSpanCapacity(ref Span);
            Debug.Assert(Span.Length >= 0, "Any QuickQueue in use should have a nonzero length Span. Was this instance default constructed without further initialization?");
        }


        public Enumerator GetEnumerator()
        {
            return new Enumerator(Span, Count, FirstIndex);
        }

        public struct Enumerator : IEnumerator<T>
        {
            private readonly TSpan span;
            private readonly int count;
            private readonly int firstIndex;
            private readonly int capacityMask;
            private int index;

            public Enumerator(TSpan span, int count, int firstIndex)
            {
                this.span = span;
                this.count = count;
                this.firstIndex = firstIndex;
                this.capacityMask = span.Length - 1;
                index = -1;
            }

            public T Current
            {
                get { return span[(firstIndex + index) & capacityMask]; }
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
    }
}
