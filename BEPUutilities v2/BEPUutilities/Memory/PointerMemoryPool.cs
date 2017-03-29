using System;
using System.Runtime.CompilerServices;

namespace BEPUutilities2.Memory
{
    /// <summary>
    /// Unmanaged memory pool that creates a
    /// </summary>
    public class BufferPool
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out RawBuffer buffer)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out RawBuffer buffer)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Return(ref RawBuffer buffer)
        {
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BufferPool<T> SpecializeFor<T>()
        {
            return new BufferPool<T>(this);
        }
    }


    /// <summary>
    /// Type specialized variants of the buffer pool are useful for use with quick collections and guaranteeing compile time type specialization.
    /// </summary>
    /// <typeparam name="T">Type of element to retrieve from the pol.</typeparam>
    public struct BufferPool<T> : IMemoryPool<T, Buffer<T>>
    {
        public readonly BufferPool Pool;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BufferPool(BufferPool pool)
        {
            this.Pool = pool;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out Buffer<T> span)
        {
            Pool.Take(count * Unsafe.SizeOf<T>(), out var rawBuffer);
            span = rawBuffer.As<T>();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out Buffer<T> span)
        {
            //Note that we can't directly use TakeForPower from the underlying pool- the actual power needed at the byte level differs!
            Pool.Take((1 << power) * Unsafe.SizeOf<T>(), out var rawBuffer);
            span = rawBuffer.As<T>();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Return(ref Buffer<T> span)
        {
            //Note that we have to rederive the original allocation size, since the size of T might not have allowed size * count to equal the original byte count.
            var rawBuffer = new RawBuffer(span.Memory, 1 << SpanHelper.GetContainingPowerOf2(Unsafe.SizeOf<T>() * span.Length));
            Pool.Return(ref rawBuffer);
        }
    }
}