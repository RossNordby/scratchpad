using BEPUutilities2.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace BEPUutilities2.ResourceManagement
{
    /// <summary>
    /// Contains references to all buffer pools created by BufferPools{T}.
    /// </summary>
    public static class BufferPools
    {
        static List<BufferPool> threadPools = new List<BufferPool>();
        static List<BufferPool> lockingPools = new List<BufferPool>();

        /// <summary>
        /// Gets the locker acquired when new pools are created by BufferPools{T}.
        /// </summary>
        public static object Locker { get; } = new object();

        /// <summary>
        /// Gets a list of all thread pools created by BufferPools{T}.
        /// Not thread safe; the list view can be corrupted by parallel operations.
        /// </summary>
        public static ReadOnlyList<BufferPool> ThreadPools
        {
            get
            {
                return new ReadOnlyList<BufferPool>(threadPools);
            }
        }

        /// <summary>
        /// Gets a list of all locking pools created by BufferPools{T}.
        /// Not thread safe; the list view can be corrupted by parallel operations.
        /// </summary>
        public static ReadOnlyList<BufferPool> LockingPools
        {
            get
            {
                return new ReadOnlyList<BufferPool>(lockingPools);
            }
        }

        /// <summary>
        /// Drops all buffer references held by all buffer pools created by BufferPools{T}.
        /// Does not affect outstanding references.
        /// Not thread safe.
        /// </summary>
        public static void Clear()
        {
            for (int i = 0; i < threadPools.Count; ++i)
            {
                threadPools[i].Clear();
            }
            for (int i = 0; i < lockingPools.Count; ++i)
            {
                lockingPools[i].Clear();
            }
        }

        internal static void AddThread<T>(UnsafeBufferPool<T> pool)
        {
            lock (Locker)
            {
                threadPools.Add(pool);
            }
        }


        internal static void AddLocking<T>(LockingBufferPool<T> pool)
        {
            lock (Locker)
            {
                lockingPools.Add(pool);
            }
        }


    }
    /// <summary>
    /// Contains locking and thread static buffer pools for the specified type.
    /// </summary>
    /// <typeparam name="T">Type of element in the buffers stored in the pools.</typeparam>
    public static class BufferPools<T>
    {
        /// <summary>
        /// Gets a buffer pool for this type which is visible by all threads.
        /// </summary>
        public static LockingBufferPool<T> Locking { get; private set; }

        [ThreadStatic]
        private static UnsafeBufferPool<T> threadPool;


        /// <summary>
        /// Gets the pool associated with this thread.
        /// </summary>
        public static UnsafeBufferPool<T> Thread
        {
            get
            {
                if (threadPool != null)
                    return threadPool;
                threadPool = new UnsafeBufferPool<T>();
                BufferPools.AddThread(threadPool);
                return threadPool;
            }
        }

        static BufferPools()
        {
            Locking = new LockingBufferPool<T>();
            BufferPools.AddLocking(Locking);
        }
    }
}
