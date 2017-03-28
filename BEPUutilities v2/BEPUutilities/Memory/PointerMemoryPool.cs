using System;
using System.Runtime.CompilerServices;

namespace BEPUutilities2.Memory
{
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