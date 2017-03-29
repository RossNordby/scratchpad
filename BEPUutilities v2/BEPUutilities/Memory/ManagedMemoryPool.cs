using System;

namespace BEPUutilities2.Memory
{
    public class ManagedMemoryPool<T> : IMemoryPool<T, Array<T>>
    {
        public void Take(int count, out Array<T> span)
        {
            throw new NotImplementedException();
        }

        public void TakeForPower(int power, out Array<T> span)
        {
            throw new NotImplementedException();
        }
        public void Return(ref Array<T> span)
        {
            throw new NotImplementedException();
        }
    }



}