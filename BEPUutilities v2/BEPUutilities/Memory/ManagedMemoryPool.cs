using System;

namespace BEPUutilities2.Memory
{
    public class ManagedMemoryPool<T> : IMemoryPool<T, ArraySpan<T>>
    {
        public void Take(int count, out ArraySpan<T> span)
        {
            throw new NotImplementedException();
        }

        public void TakeForPower(int power, out ArraySpan<T> span)
        {
            throw new NotImplementedException();
        }
        public void Return(ref ArraySpan<T> span)
        {
            throw new NotImplementedException();
        }
    }



}