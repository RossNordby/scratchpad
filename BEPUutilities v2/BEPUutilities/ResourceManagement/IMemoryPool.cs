using System;
using System.Runtime.CompilerServices;

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