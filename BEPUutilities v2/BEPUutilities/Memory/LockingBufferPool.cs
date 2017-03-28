using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace BEPUutilities2.Memory
{
    //TODO: this class should probably be entirely removed. It's only really useful when dealing with threads that weren't given a personal memory source.
    //Going forward, we should probably consider that to be a performance bug.
    /// <summary>
    /// Provides storage for reusable arrays with power-of-2 lengths.
    /// Taking resouces and returning resources are locked to prevent simultaneous access. Other functions are not locked.
    /// </summary>
    /// <typeparam name="T">Type of resource contained in the buffers.</typeparam>
    /// <remarks>This is designed for use with unsafe code. It often sacrifices safety for performance or simplicity.
    /// Running with DEBUG defined will catch some misuse, but otherwise many invalid usages will be allowed.</remarks>
    [Obsolete]
    public sealed class LockingBufferPool<T> : BufferPool<T>
    {

        SpinLock locker = new SpinLock();
        /// <summary>
        /// Gets the locker used by TakeFromPoolIndex and Return.
        /// </summary>
        public ref SpinLock Locker
        {
            get
            {
                return ref locker;
            }
        }

        /// <summary>
        /// Takes a buffer from the given pool index.
        /// </summary>
        /// <param name="poolIndex">Pool to grab a buffer from.</param>
        /// <returns>Pool of the requested size.</returns>
        public override T[] TakeFromPoolIndex(int poolIndex)
        {
            bool lockTaken = false;
            locker.Enter(ref lockTaken);
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
            bool lockTaken = false;
            locker.Enter(ref lockTaken);
            ReturnInternal(buffer, poolIndex);
            locker.Exit(false);
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
