using BEPUutilities2.ResourceManagement;
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
        ulong[] packedHandles;

        const int shift = 6;
        const int mask = 63;

        public BatchReferencedHandles(int initialCapacityInBundles)
        {
            //Remember; the bundles are 64 bodies wide. A default of 128 supports up to 8192 handles without needing resizing...
            packedHandles = new ulong[initialCapacityInBundles];
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(int handleIndex)
        {
            var packedIndex = handleIndex >> shift;
            return packedIndex < packedHandles.Length && (packedHandles[packedIndex] & (1ul << (handleIndex & mask))) > 0;
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public bool AddIfNotPresent(int handleIndex)
        //{
        //    var bundleIndex = handleIndex >> shift;
        //    if (bundleIndex >= packedHandles.Length)
        //    {
        //        //Note that we don't bother caching these arrays. For now, we treat them as too small to matter- allowing every single one to expand to the worst case size is totally fine,
        //        //and not trying to reuse them just saves some complexity.
        //        //Also note that we go ahead and jump to the next higher power of 2 rather than exactly matching the handle index. Avoids pointless resizing.
        //        //(Any waste is, again, basically ignorable.)
        //        Array.Resize(ref packedHandles, 1 << BufferPool.GetPoolIndex(handleIndex));
        //    }
        //    ref var bundle = ref packedHandles[bundleIndex];
        //    var slot = 1ul << (handleIndex & mask);
        //    var newlyAdded = (bundle & slot) == 0;
        //    //Not much point in branching to stop a single instruction that doesn't change the result.
        //    bundle |= slot;
        //    return newlyAdded;
        //}


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(int handleIndex)
        {
            var bundleIndex = handleIndex >> shift;
            if (bundleIndex >= packedHandles.Length)
            {
                //Note that we don't bother caching these arrays. For now, we treat them as too small to matter- allowing every single one to expand to the worst case size is totally fine,
                //and not trying to reuse them just saves some complexity.
                //Also note that we go ahead and jump to the next higher power of 2 rather than exactly matching the handle index. Avoids pointless resizing.
                //(Any waste is, again, basically ignorable.)
                Array.Resize(ref packedHandles, 1 << BufferPool.GetPoolIndex(handleIndex));
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
            packedHandles[handleIndex >> shift] &= ~(1ul << (handleIndex & mask));
        }
    }
}
