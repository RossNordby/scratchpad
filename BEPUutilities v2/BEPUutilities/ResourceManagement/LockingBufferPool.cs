using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace BEPUutilities.ResourceManagement
{
    /// <summary>
    /// Provides storage for reusable arrays with power-of-2 lengths.
    /// Taking resouces and returning resources are locked to prevent simultaneous access. Other functions are not locked.
    /// </summary>
    /// <typeparam name="T">Type of resource contained in the buffers.</typeparam>
    /// <remarks>This is designed for use with unsafe code. It often sacrifices safety for performance or simplicity.
    /// Running with DEBUG defined will catch some misuse, but otherwise many invalid usages will be allowed.</remarks>
    public sealed class LockingBufferPool<T> : BufferPool<T>
    {

        SpinLock locker = new SpinLock();

        /// <summary>
        /// Takes a buffer from the given pool index.
        /// </summary>
        /// <param name="poolIndex">Pool to grab a buffer from.</param>
        /// <returns>Pool of the requested size.</returns>
        public override T[] TakeFromPoolIndex(int poolIndex)
        {
            locker.Enter();
            var buffer = TakeFromPoolIndexInternal(poolIndex);
            locker.Exit();
            return buffer;
        }

        /// <summary>
        /// Releases a buffer back to the pool without clearing it out.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        /// <param name="poolIndex">Pool index associated with the buffer.</param>
        public override void Return(T[] buffer, int poolIndex)
        {
            locker.Enter();
            ReturnInternal(buffer, poolIndex);
            locker.Exit();
        }

        /// <summary>
        /// Takes a buffer from the given pool index. Does not lock.
        /// </summary>
        /// <param name="poolIndex">Pool to grab a buffer from.</param>
        /// <returns>Pool of the requested size.</returns>
        public T[] TakeFromPoolIndexUnsafe(int poolIndex)
        {
            return TakeFromPoolIndexInternal(poolIndex);
        }

        /// <summary>
        /// Releases a buffer back to the pool without clearing it out.
        /// Does not lock.
        /// </summary>
        /// <param name="buffer">Buffer to return to the pool.</param>
        /// <param name="poolIndex">Pool index associated with the buffer.</param>
        public void ReturnUnsafe(T[] buffer, int poolIndex)
        {
            ReturnInternal(buffer, poolIndex);
        }
    }
}
