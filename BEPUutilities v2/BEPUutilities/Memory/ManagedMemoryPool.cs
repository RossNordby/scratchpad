using System;

namespace BEPUutilities2.Memory
{
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



}