using BEPUutilities2.DataStructures;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BEPUutilities2.ResourceManagement
{



    /// <summary>
    /// Represents a chunk of abstract memory supporting allocations and deallocations.
    /// Never moves any memory.
    /// </summary>
    /// <remarks>Uses an extremely simple ring buffer that makes no attempt to skip groups of allocations. Not particularly efficient.</remarks>
    public class Allocator
    {

        private readonly long memoryPoolSize;

        public struct Allocation
        {
            public long Start, End;
            public ulong Previous;
            public ulong Next;
        }

        private QuickDictionary<ulong, Allocation> allocations;

        /// <summary>
        /// Creates a new memory pool.
        /// </summary>
        /// <param name="memoryPoolSize">Size of the pool in elements.</param>
        /// <param name="idBufferPool">Buffer pool to use in the allocator. If null, the allocator picks.</param>
        /// <param name="allocationBufferPool">Buffer pool to use in the allocator. If null, the allocator picks.</param>
        /// <param name="tableBufferPool">Buffer pool to use in the allocator. If null, the allocator picks.</param>
        public Allocator(long memoryPoolSize,
            BufferPool<ulong> idBufferPool = null, BufferPool<Allocation> allocationBufferPool = null, BufferPool<int> tableBufferPool = null)
        {
            this.memoryPoolSize = memoryPoolSize;
            if (idBufferPool == null)
                idBufferPool = BufferPools<ulong>.Locking;
            if (allocationBufferPool == null)
                allocationBufferPool = BufferPools<Allocation>.Locking;
            if (tableBufferPool == null)
                tableBufferPool = BufferPools<int>.Locking;
            allocations = new QuickDictionary<ulong, Allocation>(idBufferPool, allocationBufferPool, tableBufferPool);
        }

        /// <summary>
        /// Checks if the id is currently allocated.
        /// </summary>
        /// <param name="id">Id to check for.</param>
        /// <returns>True if the id is present in the allocations set, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(ulong id)
        {
            return allocations.ContainsKey(id);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddAllocation(ulong id, long start, long end,
            ref Allocation allocation, ref Allocation nextAllocation)
        {
            var newAllocation = new Allocation
            {
                Next = allocation.Next,
                Previous = nextAllocation.Previous,
                Start = start,
                End = end
            };
            //Note that the pointer modifications come BEFORE the new addition.
            //This avoids a potential pointer invalidation caused by a resize in the allocations dictionary.
            allocation.Next = id;
            nextAllocation.Previous = id;
            allocations.Add(id, newAllocation);
        }
        /// <summary>
        /// Attempts to allocate a range of memory.
        /// </summary>
        /// <param name="id">Unique id of the memory to allocate.</param>
        /// <param name="size">Size of the memory to allocate.</param>
        /// <param name="outputStart">Starting index of the allocated memory, if successful.</param>
        /// <returns>True if the allocation succeeded, false if out of memory or if memory was too fragmented to find a spot.</returns>
        public bool Allocate(ulong id, long size, out long outputStart)
        {
            Debug.Assert(!allocations.ContainsKey(id), "Id must not already be present.");
            if (allocations.Count == 0)
            {
                //If it's the first allocation, then the next and previous pointers should circle around.
                if (size < memoryPoolSize)
                {
                    outputStart = 0;
                    allocations.Add(id, new Allocation { Start = 0, End = size, Next = id, Previous = id });
                    return true;
                }
                outputStart = 0;
                return false;
            }
            //It's not the first allocation. Try to just tack it onto the end of the allocation set to begin with- it's a reasonably good place to look for empty space.
            int allocationIndex = allocations.Count - 1;
            var initialId = allocations.Keys[allocationIndex];
            while (true)
            {
                var allocation = allocations.Values[allocationIndex];
                int nextAllocationIndex = allocations.IndexOf(allocation.Next);
                var nextAllocation = allocations.Values[nextAllocationIndex];
                if (nextAllocation.Start < allocation.End)
                {
                    //Wrapped around, so the gap goes from here to the end of the memory block, and from the beginning of the memory block to the next allocation.
                    //But we need contiguous space so the two areas have to be tested independently.
                    if (memoryPoolSize - allocation.End >= size)
                    {
                        AddAllocation(id, outputStart = allocation.End, allocation.End + size, ref allocations.Values[allocationIndex], ref allocations.Values[nextAllocationIndex]);
                        return true;
                    }
                    else
                    {
                        if (nextAllocation.Start >= size)
                        {
                            AddAllocation(id, outputStart = 0, size, ref allocations.Values[allocationIndex], ref allocations.Values[nextAllocationIndex]);
                            return true;
                        }
                    }
                }
                else
                {
                    //The next allocation is in order.
                    if (nextAllocation.Start - allocation.End >= size)
                    {
                        AddAllocation(id, outputStart = allocation.End, allocation.End + size, ref allocations.Values[allocationIndex], ref allocations.Values[nextAllocationIndex]);
                        return true;
                    }

                }
                //If we get here, no open space was found.
                //Move on to the next spot.
                allocationIndex = nextAllocationIndex;

                //Have we already wrapped around?
                if (allocations.Keys[allocationIndex] == initialId)
                {
                    //Wrapped around without finding any space.
                    outputStart = 0;
                    return false;
                }
            }
        }

        /// <summary>
        /// Removes the memory associated with the id from the pool.
        /// </summary>
        /// <param name="id">Id to remove.</param>
        /// <returns>True of the id was found, false otherwise.</returns>
        public bool Deallocate(ulong id)
        {
            Allocation allocation;
            if (allocations.TryGetValue(id, out allocation))
            {
                if (allocation.Previous != id)
                {
                    var previousIndex = allocations.IndexOf(allocation.Previous);
                    Debug.Assert(allocations.Values[previousIndex].Next == id, "Previous and current must agree about their relationship.");
                    //Make the previous allocation point to the next allocation to get rid of the current allocation.
                    allocations.Values[previousIndex].Next = allocation.Next;

                    var nextIndex = allocations.IndexOf(allocation.Next);
                    Debug.Assert(allocations.Values[nextIndex].Previous == id, "Next and current must agree about their relationship.");
                    //Make the next allocation point to the previous allocation to get rid of the current allocation.
                    allocations.Values[nextIndex].Previous = allocation.Previous;
                }
                else
                {
                    Debug.Assert(allocation.Next == id, "The next index should be itself too, if previous was itself.");
                    Debug.Assert(allocations.Count == 1, "The only time where the previous allocation is itself should be when there is only a single allocation.");
                }
                allocations.FastRemove(id);
                return true;
            }
            return false;
        }

        [Conditional("DEBUG")]
        private void ValidatePointers()
        {
            if (allocations.Count == 0)
                return;
            var initialId = allocations.Keys[0];
            ulong backwardId = initialId;
            ulong forwardId = initialId;
            for (int i = 0; i < allocations.Count; ++i)
            {
                var backwardIndex = allocations.IndexOf(backwardId);
                backwardId = allocations.Values[backwardIndex].Previous;

                var forwardIndex = allocations.IndexOf(forwardId);
                forwardId = allocations.Values[forwardIndex].Next;
            }
            Debug.Assert(initialId == backwardId && initialId == forwardId, "We should be able to walk back to the starting id in exactly allocations.Count steps in either direction.");
        }
    }
}
