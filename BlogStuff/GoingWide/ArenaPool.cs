using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace GoingWide
{
    public unsafe class ArenaPool
    {
        struct Block
        {
            public byte* Pointer;
            public int Capacity;
        }

        Block[] blocks;
        int allocatedBlockCount;

        int minimumBlockSizeInBytes;

        int blockIndex;
        int indexInBlock;

        void AllocateNewBlock(int sizeInBytes)
        {
            if (blocks.Length == allocatedBlockCount)
                Array.Resize(ref blocks, blocks.Length * 2);
            blocks[allocatedBlockCount++] = new Block { Pointer = (byte*)Marshal.AllocHGlobal(sizeInBytes), Capacity = sizeInBytes };
        }

        public ArenaPool(int minimumBlockSizeInBytes = 131072, int initialAllocatedBlockCount = 1, int initialBlockCapacity = 32)
        {
            if (minimumBlockSizeInBytes <= 0)
                throw new ArgumentException("Minimum block size must be positive.");
            if (initialAllocatedBlockCount <= 0)
                throw new ArgumentException("Initial allocated block count must be positive.");
            if (initialBlockCapacity < initialAllocatedBlockCount)
                throw new ArgumentException("Initial block capacity must hold all allocated blocks.");
            this.minimumBlockSizeInBytes = minimumBlockSizeInBytes;
            blocks = new Block[initialBlockCapacity];
            for (int i = 0; i < initialAllocatedBlockCount; ++i)
            {
                AllocateNewBlock(minimumBlockSizeInBytes);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void* Allocate(int sizeInBytes)
        {
            ref var block = ref blocks[blockIndex];
            while (indexInBlock + sizeInBytes > block.Capacity)
            {
                //Move to the next block.
                ++blockIndex;
                indexInBlock = 0;
                if (blockIndex == allocatedBlockCount)
                {
                    AllocateNewBlock(sizeInBytes > minimumBlockSizeInBytes ? sizeInBytes : minimumBlockSizeInBytes);
                }
                block = ref blocks[blockIndex];
            }
            var indexToReturn = indexInBlock;
            indexInBlock += sizeInBytes;
            Debug.Assert(indexInBlock <= block.Capacity);
            return block.Pointer + indexToReturn;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Allocate(int sizeInBytes, out RawBuffer buffer)
        {
            buffer = new RawBuffer(Allocate(sizeInBytes), sizeInBytes);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Buffer<T> Allocate<T>(int instanceCount) where T : struct
        {
            return new Buffer<T>(Allocate(Unsafe.SizeOf<T>() * instanceCount), instanceCount);
        }

        /// <summary>
        /// Resets the allocation pointer to 0 without returning the underlying resources.
        /// </summary>
        public void Reset()
        {
            blockIndex = 0;
            indexInBlock = 0;
        }

        /// <summary>
        /// Resets the allocation pointer to 0 and releases underlying resources.
        /// </summary>
        public void Deallocate()
        {
            Reset();
            for (int i = 0; i < allocatedBlockCount; ++i)
            {
                Marshal.FreeHGlobal(new IntPtr(blocks[i].Pointer));
            }
            allocatedBlockCount = 0;
        }

        ~ArenaPool()
        {
            Deallocate();
        }
    }
}
