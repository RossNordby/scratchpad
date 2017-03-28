using System.Runtime.CompilerServices;

namespace BEPUutilities2.Memory
{
    public class PassthroughSpanPool<T> : IMemoryPool<T, ManagedSpan<T>>
    {
        public void Return(ref ManagedSpan<T> span)
        {
            //Drop it on the floor and let the GC deal with it.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out ManagedSpan<T> span)
        {
            TakeForPower(BufferPool.GetPoolIndex(count), out span);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out ManagedSpan<T> span)
        {
            span = new ManagedSpan<T>(new T[1 << power]);
        }
    }


}
