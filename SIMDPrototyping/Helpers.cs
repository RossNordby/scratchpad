using BEPUutilities.ResourceManagement;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public static class Helpers
    {
        public static Vector<int>[] CreateMasks()
        {
            var singleMasks = new Vector<int>[Vector<int>.Count];
            var buffer = BufferPools<int>.Locking.Take(Vector<int>.Count);
            unchecked
            {
                buffer[0] = (int)0xffffffff;
                singleMasks[0] = new Vector<int>(buffer);
                for (int i = 1; i < singleMasks.Length; ++i)
                {
                    buffer[i - 1] = 0;
                    buffer[i] = (int)0xffffffff;
                    singleMasks[i] = new Vector<int>(buffer);
                }
            }
            Array.Clear(buffer, 0, buffer.Length);
            BufferPools<int>.Locking.GiveBack(buffer);
            return singleMasks;
        }
    }
}
