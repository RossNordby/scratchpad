using System;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Represents a span of memory in a SuballocatedBufferPool.
    /// </summary>
    public struct BufferRegion
    {
        internal readonly int Power;
        /// <summary>
        /// Index of the buffer region's start in bytes.
        /// </summary>
        internal readonly int Index;

        /// <summary>
        /// Gets the length of the buffer region in bytes.
        /// </summary>
        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 1 << Power; }
        }

        /// <summary>
        /// Gets the length of the buffer region in terms of a specified type.
        /// </summary>
        /// <typeparam name="T">Type to calculate the length for.</typeparam>
        /// <returns>Length of the buffer region in terms of the given type.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetLengthForType<T>()
        {
            return Length / Unsafe.SizeOf<T>();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal BufferRegion(int power, int index)
        {
            Power = power;
            Index = index;
        }
    }

    /// <summary>
    /// Allocates buffers of data from untyped backing arrays.
    /// </summary>
    public class SuballocatedBufferPool
    {
        byte[][] memoryForPowers;
        Pow2Allocator allocator;
        /// <summary>
        /// Creates a new buffer pool.
        /// </summary>
        /// <param name="initialCapacityPerPowerInBytes">Maximum number of bytes to allocate in each power's pool initially. 
        /// If the specified capacity is fractional at a given power, the capacity is truncated to the next lower block boundary.</param>
        /// <param name="allocator">Backing allocator used to position buffers.</param>
        public SuballocatedBufferPool(int initialCapacityPerPowerInBytes, Pow2Allocator allocator)
        {
            //TODO: May be a point in extra heuristics, like maximum block count on top of maximum initial capacity.
            this.allocator = allocator;
            memoryForPowers = new byte[allocator.LargestPower][];
            for (int i = 0; i < memoryForPowers.Length; ++i)
            {
                //Don't really want to deal with both nulls and sizes as a special case, so we just create zero-count arrays if the capacity is to small to fit a whole block.
                var blockCount = initialCapacityPerPowerInBytes >> i;
                memoryForPowers[i] = new byte[blockCount << i];
            }
        }

        /// <summary>
        /// Allocates a region with the specified size.
        /// </summary>
        /// <param name="power">Size of the allocated region in terms of its power of 2. The allocated size in bytes is 2^power.</param>
        /// <param name="region">Allocated region.</param>
        /// <returns>True if the allocation required an internal resize, which invalidates any outstanding pointers or references to that power's backing memory.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Allocate(int power, out BufferRegion region)
        {
            region = new BufferRegion(power, allocator.Allocate(power));
            if (region.Index + region.Length > memoryForPowers[power].Length)
            {
                //Ouch! Didn't preallocate enough.
                //Note that the initial size could be zero (and will almost will be, for very large powers).
                //In that case, merely multiplying by two won't do anything. Need to allocate enough space for one whole region.
                Array.Resize(ref memoryForPowers[region.Power], Math.Max(1 << region.Power, memoryForPowers[region.Power].Length * 2));
                //Note that this resize invalidates any outstanding references or pointers.
                return true;
            }
            return false;
        }

        /// <summary>
        /// Frees the given region, allowing the space it used to be reused.
        /// </summary>
        /// <param name="region">Region to free.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Free(ref BufferRegion region)
        {
            allocator.Free(region.Power, region.Index);
        }

        /// <summary>
        /// Gets a reference to the start of the region.
        /// </summary>
        /// <typeparam name="T">Type to interpret the data region as.</typeparam>
        /// <param name="region">Region to grab the reference of.</param>
        /// <returns>Reference to the start of the given region.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref T GetStart<T>(ref BufferRegion region)
        {
            return ref Unsafe.As<byte, T>(ref memoryForPowers[region.Power][region.Index]);
        }

        /// <summary>
        /// Resizes the specified buffer region, copying as much data from the old region as can fit in the new one starting at the lowest index of the region.
        /// </summary>
        /// <param name="region">Region to resize. The result of the resize operation will be stored back into this reference.</param>
        /// <param name="newPower">The region's size power to allocate, where the region length is equal to 2^power.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Resize(ref BufferRegion region, int newPower)
        {
            var start = allocator.Allocate(newPower);
            Buffer.BlockCopy(memoryForPowers[region.Power], region.Index, memoryForPowers[newPower], start, Math.Min(region.Length, 1 << newPower));
            allocator.Free(region.Power, region.Index);
            region = new BufferRegion(newPower, start);
        }

        //TODO: Would be nice to have a compaction or other form of reset to drop surplus memory that isn't being used anymore.
        //On the other hand, we definitely don't need that now, and ideally you would strive to preallocate as much as possible.
    }
}
