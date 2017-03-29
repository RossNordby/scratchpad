using BEPUutilities2.Collections;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BEPUutilities2.Memory
{
    public unsafe struct PointerSpan<T> : ISpan<T> 
    {
        public readonly byte* Memory;
        int length;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PointerSpan(void* memory, int length)
        {
            Memory = (byte*)memory;
            this.length = length;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PointerSpan<T> Slice(int start, int count)
        {
            Debug.Assert(start + count < length);
            return new PointerSpan<T>(Memory + Unsafe.SizeOf<T>() * start, count);
        }

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Unsafe.Add(ref Unsafe.As<byte, T>(ref *Memory), index);
            }
        }

        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return length; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear(int start, int count)
        {
            Unsafe.InitBlock(Memory + Unsafe.SizeOf<T>() * start, 0, (uint)(count * Unsafe.SizeOf<T>()));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearManagedReferences(int start, int count)
        {
            //Pointer spans cannot validly hold managed references. nop!
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo<TOtherSpan>(int sourceStart, ref TOtherSpan targetSpan, int targetStart, int count) where TOtherSpan : ISpan<T>
        {
            if (typeof(TOtherSpan) == typeof(PointerSpan<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, PointerSpan<T>>(ref targetSpan), targetStart, count);
            }
            else if (typeof(TOtherSpan) == typeof(ArraySpan<T>))
            {
                SpanHelper.Copy(ref this, sourceStart, ref Unsafe.As<TOtherSpan, ArraySpan<T>>(ref targetSpan), targetStart, count);
            }
            else
            {
                SpanHelper.CopyFallback<T, PointerSpan<T>, TOtherSpan>(ref this, sourceStart, ref targetSpan, targetStart, count);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref T element, int start, int count)
        {
            var end = start + count;
            if (SpanHelper.IsPrimitive<T>())
            {
                var defaultComparer = default(PrimitiveComparer<T>);
                for (int i = start; start < end; ++i)
                    if (defaultComparer.Equals(ref this[i], ref element))
                        return i;
                return -1;
            }
            else
            {
                WrapperEqualityComparer<T>.CreateDefault(out var defaultComparer);
                for (int i = start; start < end; ++i)
                    if (defaultComparer.Equals(ref this[i], ref element))
                        return i;
                return -1;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element, int start, int count)
        {
            return IndexOf(ref element, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf<TPredicate>(int start, int count, ref TPredicate predicate) where TPredicate : IPredicate<T>
        {
            var end = start + count;
            for (int i = start; start < end; ++i)
                if (predicate.Matches(ref this[i]))
                    return i;
            return -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryPin(out GCHandle handle)
        {
            //We assume that the pointer span is already pinned. This isn't actually guaranteed, but... in the situations where the backing memory isn't pinned, there's a bug.
            handle = new GCHandle();
            return false;
        }
    }

}