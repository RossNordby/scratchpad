using System.Runtime.CompilerServices;

namespace BEPUutilities2.Memory
{
    public class PassthroughSpanPool<T> : IMemoryPool<T, ArraySpan<T>>
    {
        public void Return(ref ArraySpan<T> span)
        {
            //Drop it on the floor and let the GC deal with it.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out ArraySpan<T> span)
        {
            TakeForPower(BufferPool.GetPoolIndex(count), out span);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out ArraySpan<T> span)
        {
            span = new ArraySpan<T>(new T[1 << power]);
        }
    }


}
