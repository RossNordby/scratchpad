using BEPUutilities2.Collections;
using System;
using System.Collections.Generic;

namespace BEPUutilities2.Memory
{
    /// <summary>
    /// Manages a pool of identifier values. Grabbing an id from the pool picks a number that has been picked and returned before, 
    /// or if none of those are available, the minimum value greater than any existing id.
    /// </summary>
    public class IdPool
    {
        private int nextIndex;

        /// <summary>
        /// Gets the highest value which any index claimed thus far could possibly have.
        /// This is not necessarily the current highest claimed index; this value may represent an earlier claim that has already been released.
        /// -1 if nothing has ever been claimed.
        /// </summary>
        public int HighestPossiblyClaimedId
        {
            get { return nextIndex - 1; }
        }
        BufferPool<int> pool;
        public IdPool(BufferPool pool, int initialCapacity = 128)
        {
            this.pool = pool.SpecializeFor<int>();
            QuickQueue<int, Buffer<int>>.Create(this.pool, initialCapacity, out AvailableIds);
        }

        //Note that all availableIds are guaranteed to be less than nextIndex.
        //[0, nextIndex) contains all currently used ids and ids contained within availableIds.
        public QuickQueue<int, Buffer<int>> AvailableIds;

        public int Take()
        {
            if (AvailableIds.TryDequeue(out var id))
                return id;
            return nextIndex++;
        }

        public void Return(int id)
        {
            AvailableIds.Enqueue(id, pool);
        }

        /// <summary>
        /// Resets the IdPool without returning any resources to the underlying memory pool.
        /// </summary>
        public void Clear()
        {
            nextIndex = 0;
            AvailableIds.FastClear();
        }

        /// <summary>
        /// Ensures that the underlying id queue can hold at least a certain number of ids.
        /// </summary>
        /// <param name="queuedCount">Number of elements to preallocate space for in the available ids queue.</param>
        public void EnsureCapacity(int queuedCount)
        {
            AvailableIds.EnsureCapacity(queuedCount, pool);
        }

        /// <summary>
        /// Shrinks the available ids queue to the smallest size that can fit the given count and the current available id count.
        /// </summary>
        /// <param name="queuedCount">Number of elements to guarantee space for in the available ids queue.</param>
        public void Compact(int queuedCount)
        {
            var targetLength = BufferPool<int>.GetLowestContainingElementCount(Math.Max(queuedCount, AvailableIds.Count));
            if (AvailableIds.Span.Length > targetLength)
            {
                AvailableIds.Resize(targetLength, pool);
            }
        }
        /// <summary>
        /// Resizes the underlying buffer to the smallest size required to hold the given count and the current available id count.
        /// </summary>
        /// <param name="queuedCount">Number of elements to guarantee space for in the available ids queue.</param>
        public void Resize(int queuedCount)
        {
            var targetLength = BufferPool<int>.GetLowestContainingElementCount(Math.Max(queuedCount, AvailableIds.Count));
            if (AvailableIds.Span.Length != targetLength)
            {
                AvailableIds.Resize(targetLength, pool);
            }
        }

        /// <summary>
        /// Returns underlying memory to the pool.
        /// </summary>
        /// <remarks>The IdPool can be reused only if EnsureCapacity or Resize is called.</remarks>
        public void Dispose()
        {
            AvailableIds.Dispose(pool);
            //This simplifies reuse and makes it harder to use invalid data.
            nextIndex = 0;
            AvailableIds = new QuickQueue<int, Buffer<int>>();
        }

    }
}
