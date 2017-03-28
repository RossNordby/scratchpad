using BEPUutilities2.Collections;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BEPUutilities2.Memory
{
    public struct ManagedSpan<T> : ISpan<T>
    {
        public readonly T[] Array;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ManagedSpan(T[] array)
        {
            this.Array = array;
        }

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Array[index];
            }
        }
        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return Array.Length;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear(int start, int count)
        {
            System.Array.Clear(Array, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearManagedReferences(int start, int count)
        {
            //TODO: Should check to see if it is primitive first; that's something the jit can do at compile time.
            //Can't easily check to see if it contains *any* references recursively at compile time, though- that's trickier.
            System.Array.Clear(Array, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo<TOtherSpan>(int sourceStart, ref TOtherSpan targetSpan, int targetStart, int count) where TOtherSpan : ISpan<T>
        {
            //TODO: Check for jit specialization
            if (typeof(TOtherSpan) == typeof(ManagedSpan<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, ManagedSpan<T>>(ref targetSpan), targetStart, count);
            }
            else if (typeof(TOtherSpan) == typeof(PointerSpan<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, PointerSpan<T>>(ref targetSpan), targetStart, count);
            }
            else
            {
                SpanHelper.CopyFallback<T, ManagedSpan<T>, TOtherSpan>(ref this, sourceStart, ref targetSpan, targetStart, count);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element, int start, int count)
        {
            return System.Array.IndexOf(Array, element, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref T element, int start, int count)
        {
            return System.Array.IndexOf(Array, element, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf<TPredicate>(int start, int count, ref TPredicate predicate) where TPredicate : IPredicate<T>
        {
            var end = start + count;
            for (int i = start; i < end; ++i)
            {
                if (predicate.Matches(ref Array[i]))
                    return i;
            }
            return -1;
        }

        public bool TryPin(out GCHandle handle)
        {
            handle = GCHandle.Alloc(Array, GCHandleType.Pinned);
            return true;
        }
    }



}