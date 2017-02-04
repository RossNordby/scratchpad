#define OWNERSHIPTRACKING
using BEPUutilities2.ResourceManagement;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Simplistic allocation tracker for power of 2 sized regions. Assumes that every power has its own array.
    /// Memory limits not tracked; assumes the user will handle any resizing requirements.
    /// </summary>
    public class Pow2Allocator
    {
        //Note that the separation of pools makes this a little more wasteful than a single array suballocation scheme.
        //In return for some memory waste, we gain a great deal of simplicity. 
        //Since the memory waste is strictly a capacity concern and not a bandwidth concern, this is a good trade.

        //Note that this closely matches the behavior of the old BufferPools implementation, except it's designed with suballocation from large preallocated arrays in mind.
        //That can significantly reduce reference counts.

        //Note that this does not include any actual arrays. This only handles the conceptual level of the allocations, leaving details of suballocation up to the user.
        //(An implementation might use a Span<T> or pinning with pointers over a backing byte array to avoid per-type memory waste, for example.)
        IdPool[] pools;
        public readonly int LargestPower;
        public Pow2Allocator(int largestPower = 31, int initialBlocksPerPower = 128)
        {
            LargestPower = largestPower;
            pools = new IdPool[largestPower];
            for (int i = 0; i < largestPower; ++i)
            {
                pools[i] = new IdPool(initialBlocksPerPower);
            }
        }

#if OWNERSHIPTRACKING && DEBUG
        struct Allocation
        {
            public int Power, Index;
            public override int GetHashCode()
            {
                //Not too concerned about a good hash here.
                return Power * 17 + Index * 37;
            }
        }
        HashSet<Allocation> outstandingAllocations = new HashSet<Allocation>();
#endif

        /// <summary>
        /// Allocates a block for the given power. Index returned is in terms of elements, not blocks.
        /// </summary>
        /// <param name="power">The exponent associated with the size of the allocation. That is, if the size is 32, 'power' is 5.</param>
        /// <returns>Index of the allocated block in terms of elements.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Allocate(int power)
        {
            var index = pools[power].Take() << power;
#if OWNERSHIPTRACKING && DEBUG
            var added = outstandingAllocations.Add(new Allocation { Power = power, Index = index });
            Debug.Assert(added, "Can only add things which were not already present.");
#endif
            return index;
        }

        /// <summary>
        /// Frees a block.
        /// </summary>
        /// <param name="power">The exponent associated with the size of the allocation. That is, if the size is 32, 'power' is 5.</param>
        /// <param name="index">Index of the allocation to free.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Free(int power, int index)
        {
#if OWNERSHIPTRACKING && DEBUG
            var removed = outstandingAllocations.Remove(new Allocation { Power = power, Index = index });
            Debug.Assert(removed, "Can only free things which were present.");
#endif
            pools[power].Return(index >> power);
        }
    }
}
