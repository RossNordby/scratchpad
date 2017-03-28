using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BEPUutilities2.ResourceManagement
{
    /// <summary>
    /// Defines a type that is capable of pooling blocks of memory in the form of ISpan{T}.
    /// The backing memory may be untyped.
    /// </summary>
    public interface IMemoryPool<T, TSpan> where TSpan : ISpan<T>
    {
        void Take(int count, out TSpan span);
        void TakeForPower(int power, out TSpan span);
        void Return(ref TSpan span);
    }
    public struct ManagedSpan<T> : ISpan<T>
    {
        readonly T[] array;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ManagedSpan(T[] array)
        {
            this.array = array;
        }

        public ref T this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref array[index];
            }
        }
        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return array.Length;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear(int start, int count)
        {
            Array.Clear(array, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ClearManagedReferences(int start, int count)
        {
            //TODO: Should check to see if it is primitive first; that's something the jit can do at compile time.
            //Can't easily check to see if it contains *any* references recursively at compile time, though- that's trickier.
            Array.Clear(array, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyTo<TOtherSpan>(int sourceStart, ref TOtherSpan targetSpan, int targetStart, int count) where TOtherSpan : ISpan<T>
        {
            //TODO: Check for jit specialization
            if(typeof(TOtherSpan) == typeof(ManagedSpan<T>))
            {
                Array.Copy(array, sourceStart, Unsafe.As<TOtherSpan, ManagedSpan<T>>(ref targetSpan).array, targetStart, count);
            }
            else
            {
                
                Debug.Assert(sourceStart + count <= Length && targetStart + count < targetSpan.Length);
                var sourceEnd = sourceStart + count;
                var sourceIndex = sourceStart;
                var targetIndex = targetStart;
                while(sourceIndex < sourceEnd)
                {
                    targetSpan[targetIndex++] = array[targetIndex++];
                }
            }
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(T element, int start, int count)
        {
            return Array.IndexOf(array, element, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(ref T element, int start, int count)
        {
            return Array.IndexOf(array, element, start, count);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf<TPredicate>(int start, int count, ref TPredicate predicate) where TPredicate : IPredicate<T>
        {
            var end = start + count;
            for (int i = start; i < end; ++i)
            {
                if (predicate.Matches(ref array[i]))
                    return i;
            }
            return -1;
        }
    }
    public struct PointerSpan<T> : ISpan<T>
    {
        public ref T this[int index] => throw new NotImplementedException();

        public int Length => throw new NotImplementedException();

        public void Clear(int start, int count)
        {
            throw new NotImplementedException();
        }

        public void ClearManagedReferences(int start, int count)
        {
            throw new NotImplementedException();
        }

        public void CopyTo<TOtherSpan>(int sourceStart, ref TOtherSpan targetSpan, int targetStart, int count) where TOtherSpan : ISpan<T>
        {
            throw new NotImplementedException();
        }

        public int IndexOf(T element, int start, int count)
        {
            throw new NotImplementedException();
        }

        public int IndexOf(ref T element, int start, int count)
        {
            throw new NotImplementedException();
        }

        public int IndexOf<TPredicate>(int start, int count, ref TPredicate predicate) where TPredicate : IPredicate<T>
        {
            throw new NotImplementedException();
        }
    }

    public class ManagedMemoryPool<T> : IMemoryPool<T, ManagedSpan<T>>
    {
        public void Take(int count, out ManagedSpan<T> span)
        {
            throw new NotImplementedException();
        }

        public void TakeForPower(int power, out ManagedSpan<T> span)
        {
            throw new NotImplementedException();
        }
        public void Return(ref ManagedSpan<T> span)
        {
            throw new NotImplementedException();
        }
    }
    public class PointerMemoryPool
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take<T>(int count, out PointerSpan<T> span)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower<T>(int power, out PointerSpan<T> span)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return<T>(ref PointerSpan<T> span)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PointerMemoryPool<T> SpecializeFor<T>()
        {
            return new PointerMemoryPool<T>(this);
        }
    }


    public struct PointerMemoryPool<T> : IMemoryPool<T, PointerSpan<T>>
    {
        readonly PointerMemoryPool pool;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PointerMemoryPool(PointerMemoryPool pool)
        {
            this.pool = pool;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out PointerSpan<T> span)
        {
            pool.Take(count, out span);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int count, out PointerSpan<T> span)
        {
            pool.TakeForPower(count, out span);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(ref PointerSpan<T> span)
        {
            pool.Return(ref span);
        }
    }

}