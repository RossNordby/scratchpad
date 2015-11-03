using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace BEPUutilities2.ResourceManagement
{
    /// <summary>
    /// Offers the interface of a buffer pool, but creates resources on demand and never stores them.
    /// Useful when a "Quick" collection desires more standard "drop it and let the GC handle it" behavior.
    /// </summary>
    /// <typeparam name="T">Type of resource contained in the buffers.</typeparam>
    public sealed class PassthroughBufferPool<T> : BufferPool<T>
    {
       

        /// <summary>
        /// Takes a buffer from the given pool index.
        /// </summary>
        /// <param name="poolIndex">Pool to grab a buffer from.</param>
        /// <returns>Pool of the requested size.</returns>
        public override T[] TakeFromPoolIndex(int poolIndex)
        {
            return new T[1 << poolIndex];
        }
        
        /// <summary>
        /// Releases a buffer back to the pool without clearing it out.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        /// <param name="poolIndex">Pool index associated with the buffer.</param>
        public override void Return(T[] buffer, int poolIndex)
        {
        }
    }
}
