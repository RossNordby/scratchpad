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
        //TODO: Note that we just use unpinned managed arrays as backing memory at the moment. This may change later in favor of a pointer-compatible representation.
        //One likely candidate is to simply pin the arrays that are created. These arrays will tend to be so large that the GC won't compact them anyway (barring an explicit order).
        //We could also just pull some memory from an unmanaged heap. The memory source could just be abstracted to a provider- it doesn't really matter where it comes from, so long
        //as it supports pinning (should we implement the pointer based approach).
        byte[][] memoryForPowers;
        Pow2Allocator allocator;

        private SuballocatedBufferPool(Pow2Allocator allocator)
        {
            this.allocator = allocator;
            memoryForPowers = new byte[allocator.LargestPower][];
        }
        /// <summary>
        /// Creates a new buffer pool.
        /// </summary>
        /// <param name="initialCapacityPerPowerInBytes">Maximum number of bytes to allocate in each power's pool initially. 
        /// If the specified capacity is fractional at a given power, the capacity is truncated to the next lower block boundary.</param>
        /// <param name="allocator">Backing allocator used to position buffers.</param>
        public SuballocatedBufferPool(int initialCapacityPerPowerInBytes, Pow2Allocator allocator)
            : this(allocator)
        {
            for (int i = 0; i < memoryForPowers.Length; ++i)
            {
                var blockCount = initialCapacityPerPowerInBytes >> i;
                memoryForPowers[i] = new byte[blockCount << i];
            }
        }
        /// <summary>
        /// Creates a new buffer pool.
        /// </summary>
        /// <param name="initialCapacityPerPowerInBytes">Function that returns the number of bytes preallocated for a given power.
        /// If the specified capacity is fractional at a given power, the capacity is truncated to the next lower block boundary.</param>
        /// <param name="allocator">Backing allocator used to position buffers.</param>
        public SuballocatedBufferPool(Func<int, int> initialCapacityPerPowerInBytes, Pow2Allocator allocator)
            : this(allocator)
        {
            for (int i = 0; i < memoryForPowers.Length; ++i)
            {
                var blockCount = initialCapacityPerPowerInBytes(i) >> i;
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
                //TODO: This kind of sneaky invalidation causes a lot of problems. We can't store a direct pointer or any equivalent representation,
                //because it would potentially get the rug pulled out from beneath it. Options:
                //1) Preallocate a huge amount and don't support resizes. Technically workable, but fragile and requires user awareness to specify preallocation. 
                //Would rather not impose that.
                //2) Rather than recreating the array and dropping the previous one, 'resize' simply allocates another set of blocks in a new array.
                //Each new blockset would be the same size, so indexing for the purposes of freeing would be easy (shift and mask).
                //In the limit, this approaches the BEPUutilities v1 BufferPool, which simply used a separate managed array for each 'block'.
                //The advantage of the chained pools relative to the v1 approach would be fewer references (especially in the small array cases) 
                //and a single backing type shared by all suballocators.
                //3) Maintain outstanding bufferregion references, and push an update out to them when a resize occurs. 
                //This gets complicated and pretty much destroys the entire point of the BufferRegion for purposes of efficient referenceless storage.

                //I suspect we'll end up doing #2 at some point, along with moving to a pointer-backed representation for a cheaper GetStart<T>().
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
        /// Gets a reference to the start of the region. Potentially invalidated by any allocation that induces a resize.
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
            Allocate(newPower, out var newRegion);
            Buffer.BlockCopy(memoryForPowers[region.Power], region.Index, memoryForPowers[newPower], newRegion.Index, Math.Min(region.Length, 1 << newPower));
            allocator.Free(region.Power, region.Index);
            region = newRegion;
        }

        //TODO: Would be nice to have a compaction or other form of reset to drop surplus memory that isn't being used anymore.
        //On the other hand, we definitely don't need that now, and ideally you would strive to preallocate as much as possible.
    }
}
