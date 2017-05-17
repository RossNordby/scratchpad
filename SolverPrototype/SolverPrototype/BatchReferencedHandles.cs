using BEPUutilities2.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Contains the set of body handles referenced by a constraint batch.
    /// </summary>
    struct BatchReferencedHandles
    {
        /// <summary>
        /// Since we only ever need to support add, remove and contains checks, and because body handles are guaranteed unique,
        /// we can just use packed bitfields. Each bit represents one body handle's containment state.
        /// </summary>
        /// <remarks> 
        /// This can grow up to the number of (bodies / 8) bytes in the worst case, but that is much, much smaller than using a dictionary or set.
        /// 16384 bodies would only take 2KB. Even if you have 1000 batches all at that size, it's a pretty unconcerning amount of storage.
        /// (And to be clear, 1000 batches is a crazy pathological number. Most simulations will have less than 20 batches.)
        /// </remarks>
        Buffer<ulong> packedHandles;

        const int shift = 6;
        const int mask = 63;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int GetSizeInLongs(int count)
        {
            return (count >> shift) + ((count & mask) > 0 ? 1 : 0);
        }

        public BatchReferencedHandles(BufferPool pool, int initialHandleCapacity)
        {
            //Remember; the bundles are 64 bodies wide. A default of 128 supports up to 8192 handles without needing resizing...
            pool.SpecializeFor<ulong>().Take(GetSizeInLongs(initialHandleCapacity), out packedHandles);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(int handleIndex)
        {
            var packedIndex = handleIndex >> shift;
            return packedIndex < packedHandles.Length && (packedHandles[packedIndex] & (1ul << (handleIndex & mask))) > 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(int handleIndex, BufferPool pool)
        {
            var bundleIndex = handleIndex >> shift;
            if (bundleIndex >= packedHandles.Length)
            {
                pool.SpecializeFor<ulong>().Resize(ref packedHandles, 1 << SpanHelper.GetContainingPowerOf2(bundleIndex + 1), packedHandles.Length);
            }
            ref var bundle = ref packedHandles[bundleIndex];
            var slot = 1ul << (handleIndex & mask);
            Debug.Assert((bundle & slot) == 0, "Cannot add if it's already present!");
            //Not much point in branching to stop a single instruction that doesn't change the result.
            bundle |= slot;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(int handleIndex)
        {
            Debug.Assert((packedHandles[handleIndex >> shift] & (1ul << (handleIndex & mask))) > 0, "If you remove a handle, it should be present.");
            packedHandles[handleIndex >> shift] &= ~(1ul << (handleIndex & mask));
        }

        public void Clear()
        {
            packedHandles.Clear(0, packedHandles.Length);
        }

        public void EnsureCapacity(int handleCount, BufferPool pool)
        {
            var desiredSize = GetSizeInLongs(handleCount);
            if (packedHandles.Length < desiredSize)
            {
                pool.SpecializeFor<ulong>().Resize(ref packedHandles, desiredSize, packedHandles.Length);
            }
        }

        //While we expose a compaction and resize, using it requires care. It would be a mistake to shrink beyond the current bodies handles size.
        public void Compact(int handleCount, BufferPool pool)
        {
            var desiredSize = BufferPool<ulong>.GetLowestContainingElementCount(GetSizeInLongs(handleCount));
            if (packedHandles.Length > desiredSize)
            {
                pool.SpecializeFor<ulong>().Resize(ref packedHandles, desiredSize, packedHandles.Length);
            }
        }
        public void Resize(int handleCount, BufferPool pool)
        {
            var desiredSize = BufferPool<ulong>.GetLowestContainingElementCount(GetSizeInLongs(handleCount));
            if (packedHandles.Length != desiredSize)
            {
                pool.SpecializeFor<ulong>().Resize(ref packedHandles, desiredSize, packedHandles.Length);
            }
        }
        /// <summary>
        /// Disposes the internal buffer.
        /// </summary>
        /// <remarks>The instance can be reused after a Dispose if EnsureCapacity or Resize is called.
        /// That's a little meaningless given that the instance is a value type, but hey, you don't have to new another one, that's something.</remarks>
        public void Dispose(BufferPool pool)
        {
            Debug.Assert(packedHandles.Length > 0, "Cannot double-dispose.");
            pool.SpecializeFor<ulong>().Return(ref packedHandles);
            packedHandles = new Buffer<ulong>();
        }
    }
}
