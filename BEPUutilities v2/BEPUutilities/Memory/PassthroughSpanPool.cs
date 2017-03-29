using System.Runtime.CompilerServices;

namespace BEPUutilities2.Memory
{
    public class PassthroughSpanPool<T> : IMemoryPool<T, Array<T>>
    {
        public void Return(ref Array<T> span)
        {
            //Drop it on the floor and let the GC deal with it.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out Array<T> span)
        {
            TakeForPower(BufferPool.GetPoolIndex(count), out span);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out Array<T> span)
        {
            span = new Array<T>(new T[1 << power]);
        }
    }


}
