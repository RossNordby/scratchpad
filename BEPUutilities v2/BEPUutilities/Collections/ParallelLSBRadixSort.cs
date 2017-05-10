using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace BEPUutilities2.Collections
{
    public unsafe class ParallelLSBRadixSort
    {
        //TODO: If the jit ever managed to handle ISpan indexers optimally, we could use a much more natural ISpan-based implementation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ReorderForByte<T>(ref uint sourceKeys, ref uint targetKeys, ref T sourceValues, ref T targetValues, int keyCount, ref int bucketStartIndices, int shift)
        {
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref sourceKeys, i);
                ref var bucketStartIndex = ref Unsafe.Add(ref bucketStartIndices, (int)(key >> shift) & 0xFF);
                Unsafe.Add(ref targetKeys, bucketStartIndex) = key;
                Unsafe.Add(ref targetValues, bucketStartIndex) = Unsafe.Add(ref sourceValues, i);
                //Bump the index up to compensate for the new element.
                ++bucketStartIndex;
            }
        }

        Buffer<uint> keys;
        int keyCount;
        int nextWorkBlockIndex;
        int workBlockSize;
        struct Worker
        {
            public Buffer<int> Buckets0;
            public Buffer<int> Buckets8;
            public Buffer<int> Buckets16;
            public Buffer<int> Buckets24;
        }
        Buffer<int> bucketStarts0;
        Buffer<int> bucketStarts8;
        Buffer<int> bucketStarts16;
        Buffer<int> bucketStarts24;

        Buffer<Worker> workers;
        void Fill4Buckets(int workerIndex)
        {
            ref var worker = ref workers[workerIndex];

            //We assume all the bucket counts start at zero. Enforce that.
            uint bucketSize = (uint)(worker.Buckets0.Length * sizeof(int));
            Unsafe.InitBlock(worker.Buckets0.Memory, 0, bucketSize);
            Unsafe.InitBlock(worker.Buckets8.Memory, 0, bucketSize);
            Unsafe.InitBlock(worker.Buckets16.Memory, 0, bucketSize);
            Unsafe.InitBlock(worker.Buckets24.Memory, 0, bucketSize);

            ref var firstKey = ref keys[0];
            while (true)
            {
                var workBlockIndex = Interlocked.Increment(ref nextWorkBlockIndex) - 1;
                var index = workBlockIndex * workBlockSize;
                while (true)
                {
                    if (index >= keyCount)
                        return;

                    var key = Unsafe.Add(ref firstKey, index);
                    ++worker.Buckets0[(int)(key & 0xFF)];
                    ++worker.Buckets8[(int)((key >> 8) & 0xFF)];
                    ++worker.Buckets16[(int)((key >> 16) & 0xFF)];
                    ++worker.Buckets24[(int)((key >> 24))];
                }
            }
        }
        void Fill3Buckets(int workerIndex)
        {
            ref var worker = ref workers[workerIndex];

            //We assume all the bucket counts start at zero. Enforce that.
            uint bucketSize = (uint)(worker.Buckets0.Length * sizeof(int));
            Unsafe.InitBlock(worker.Buckets0.Memory, 0, bucketSize);
            Unsafe.InitBlock(worker.Buckets8.Memory, 0, bucketSize);
            Unsafe.InitBlock(worker.Buckets16.Memory, 0, bucketSize);

            ref var firstKey = ref keys[0];
            while (true)
            {
                var workBlockIndex = Interlocked.Increment(ref nextWorkBlockIndex) - 1;
                var index = workBlockIndex * workBlockSize;
                while (true)
                {
                    if (index >= keyCount)
                        return;

                    var key = Unsafe.Add(ref firstKey, index);
                    ++worker.Buckets0[(int)(key & 0xFF)];
                    ++worker.Buckets8[(int)((key >> 8) & 0xFF)];
                    ++worker.Buckets16[(int)((key >> 16) & 0xFF)];
                }
            }
        }
        void Fill2Buckets(int workerIndex)
        {
            ref var worker = ref workers[workerIndex];

            //We assume all the bucket counts start at zero. Enforce that.
            uint bucketSize = (uint)(worker.Buckets0.Length * sizeof(int));
            Unsafe.InitBlock(worker.Buckets0.Memory, 0, bucketSize);
            Unsafe.InitBlock(worker.Buckets8.Memory, 0, bucketSize);


            ref var firstKey = ref keys[0];
            while (true)
            {
                var workBlockIndex = Interlocked.Increment(ref nextWorkBlockIndex) - 1;
                var index = workBlockIndex * workBlockSize;
                if (index >= keyCount)
                    break;
                var end = Math.Min(keyCount, index + workBlockSize);
                while (true)
                {
                    var key = Unsafe.Add(ref firstKey, index);
                    ++worker.Buckets0[(int)(key & 0xFF)];
                    ++worker.Buckets8[(int)((key >> 8) & 0xFF)];
                    ++index;
                    if (index >= end)
                        break;
                }
            }
        }
        void Fill1Bucket(int workerIndex)
        {
            ref var worker = ref workers[workerIndex];

            //We assume all the bucket counts start at zero. Enforce that.
            uint bucketSize = (uint)(worker.Buckets0.Length * sizeof(int));
            Unsafe.InitBlock(worker.Buckets0.Memory, 0, bucketSize);

            ref var firstKey = ref keys[0];
            while (true)
            {
                var workBlockIndex = Interlocked.Increment(ref nextWorkBlockIndex) - 1;
                var index = workBlockIndex * workBlockSize;
                while (true)
                {
                    if (index >= keyCount)
                        return;

                    var key = Unsafe.Add(ref firstKey, index);
                    ++worker.Buckets0[(int)(key & 0xFF)];
                }
            }
        }

        /// <summary>
        /// Sorts a set of keys and their associated values using a parallel radix sort.
        /// </summary>
        /// <remarks>Only one invocation of the sort can be running at a time on a given instance of the sorter.</remarks>
        /// <typeparam name="TValue">Type of the values to sort.</typeparam>
        /// <typeparam name="TKeySpan">Type of the span that holds the keys to sort.</typeparam>
        /// <typeparam name="TValueSpan">Type of the span that holds the values to sort.</typeparam>
        /// <param name="keys">Span containing the keys to sort.</param>
        /// <param name="values">Span containing the values to sort.</param>
        /// <param name="startIndex">Start location of the sort.</param>
        /// <param name="count">Number of elements, including the start index, to sort.</param>
        /// <param name="keysUpperBound">Value equal to or greater than the value of any key within the sorted key set. Tighter bounds can allow faster execution.</param>
        /// <param name="keysScratch">Scratch array to write temporary results into.</param>
        /// <param name="valuesScratch">Scratch array to write temporary results into.</param>
        /// <param name="bufferPool">Pool to pull temporary buffers from.</param>
        /// <param name="sortedKeys">Span containing the sorted keys. Will be either the input keys span or keysScratch span depending on the keysUpperBound.</param>
        /// <param name="sortedValues">Span containing the sorted values. Will be either the input values span or valuesScratch span depending on the keysUpperBound.</param>
        public unsafe void Sort<TValue>(ref Buffer<uint> keys, ref Buffer<TValue> values, int startIndex, int count, uint keysUpperBound,
            ref Buffer<uint> keysScratch, ref Buffer<TValue> valuesScratch, BufferPool bufferPool, IThreadDispatcher threadDispatcher,
            out Buffer<uint> sortedKeys, out Buffer<TValue> sortedValues)
        {
            //We create a set of buckets in a single pass across all keys. We have to know how many levels of buckets to fill.
            int bucketSetCount = keysUpperBound < (1 << 16) ? keysUpperBound < (1 << 8) ? 1 : 2 : keysUpperBound < (1 << 24) ? 3 : 4;
            var intPool = bufferPool.SpecializeFor<int>();
            //Every thread requires its own bucket set.
            bufferPool.SpecializeFor<Worker>().Take(threadDispatcher.ThreadCount, out workers);
            const int bucketCount = 256;
            intPool.Take(bucketCount, out bucketStarts0);
            if (bucketSetCount >= 2) intPool.Take(bucketCount, out bucketStarts8);
            if (bucketSetCount >= 3) intPool.Take(bucketCount, out bucketStarts16);
            if (bucketSetCount >= 4) intPool.Take(bucketCount, out bucketStarts24);
            for (int workerIndex = 0; workerIndex < threadDispatcher.ThreadCount; ++workerIndex)
            {
                intPool.Take(bucketCount, out workers[workerIndex].Buckets0);
                if (bucketSetCount >= 2) intPool.Take(bucketCount, out workers[workerIndex].Buckets8);
                if (bucketSetCount >= 3) intPool.Take(bucketCount, out workers[workerIndex].Buckets16);
                if (bucketSetCount >= 4) intPool.Take(bucketCount, out workers[workerIndex].Buckets24);
            }
            nextWorkBlockIndex = 0;
            workBlockSize = Math.Min(Math.Max(count / threadDispatcher.ThreadCount, 1), 64);
            keyCount = count;
            this.keys = keys.Slice(startIndex, count);
            switch (bucketSetCount)
            {
                case 1:
                    {
                        threadDispatcher.DispatchWorkers(Fill1Bucket);
                        //Compute the start indices for each bucket.
                        ref var start0 = ref bucketStarts0[0];
                        int previousSum0 = 0;
                        for (int bucketIndex = 0; bucketIndex < bucketCount; ++bucketIndex)
                        {
                            Unsafe.Add(ref start0, bucketIndex) = previousSum0;
                            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                            {
                                ref var worker = ref workers[i];
                                previousSum0 += worker.Buckets0[bucketIndex];
                            }
                        }
                        ReorderForByte(ref keys[0], ref keysScratch[0], ref values[0], ref valuesScratch[0], keyCount, ref bucketStarts0[0], 0);
                        sortedKeys = keysScratch;
                        sortedValues = valuesScratch;
                    }
                    break;
                case 2:
                    {
                        threadDispatcher.DispatchWorkers(Fill2Buckets);
                        //Compute the start indices for each bucket.
                        ref var start0 = ref bucketStarts0[0];
                        ref var start8 = ref bucketStarts8[0];
                        int previousSum0 = 0;
                        int previousSum8 = 0;
                        for (int bucketIndex = 0; bucketIndex < bucketCount; ++bucketIndex)
                        {
                            Unsafe.Add(ref start0, bucketIndex) = previousSum0;
                            Unsafe.Add(ref start8, bucketIndex) = previousSum8;
                            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                            {
                                ref var worker = ref workers[i];
                                previousSum0 += worker.Buckets0[bucketIndex];
                                previousSum8 += worker.Buckets8[bucketIndex];
                            }
                        }
                        ReorderForByte(ref keys[0], ref keysScratch[0], ref values[0], ref valuesScratch[0], keyCount, ref bucketStarts0[0], 0);
                        ReorderForByte(ref keysScratch[0], ref keys[0], ref valuesScratch[0], ref values[0], keyCount, ref bucketStarts8[0], 8);
                        sortedKeys = keys;
                        sortedValues = values;
                    }
                    break;
                case 3:
                    {
                        threadDispatcher.DispatchWorkers(Fill3Buckets);
                        //Compute the start indices for each bucket.
                        ref var start0 = ref bucketStarts0[0];
                        ref var start8 = ref bucketStarts8[0];
                        ref var start16 = ref bucketStarts16[0];
                        int previousSum0 = 0;
                        int previousSum8 = 0;
                        int previousSum16 = 0;
                        for (int bucketIndex = 0; bucketIndex < bucketCount; ++bucketIndex)
                        {
                            Unsafe.Add(ref start0, bucketIndex) = previousSum0;
                            Unsafe.Add(ref start8, bucketIndex) = previousSum8;
                            Unsafe.Add(ref start16, bucketIndex) = previousSum16;
                            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                            {
                                ref var worker = ref workers[i];
                                previousSum0 += worker.Buckets0[bucketIndex];
                                previousSum8 += worker.Buckets8[bucketIndex];
                                previousSum16 += worker.Buckets16[bucketIndex];
                            }
                        }
                        ReorderForByte(ref keys[0], ref keysScratch[0], ref values[0], ref valuesScratch[0], keyCount, ref bucketStarts0[0], 0);
                        ReorderForByte(ref keysScratch[0], ref keys[0], ref valuesScratch[0], ref values[0], keyCount, ref bucketStarts8[0], 8);
                        ReorderForByte(ref keys[0], ref keysScratch[0], ref values[0], ref valuesScratch[0], keyCount, ref bucketStarts16[0], 16);
                        sortedKeys = keysScratch;
                        sortedValues = valuesScratch;
                    }
                    break;
                case 4:
                    {
                        threadDispatcher.DispatchWorkers(Fill4Buckets);
                        //Compute the start indices for each bucket.
                        ref var start0 = ref bucketStarts0[0];
                        ref var start8 = ref bucketStarts8[0];
                        ref var start16 = ref bucketStarts16[0];
                        ref var start24 = ref bucketStarts24[0];
                        int previousSum0 = 0;
                        int previousSum8 = 0;
                        int previousSum16 = 0;
                        int previousSum24 = 0;
                        for (int bucketIndex = 0; bucketIndex < bucketCount; ++bucketIndex)
                        {
                            Unsafe.Add(ref start0, bucketIndex) = previousSum0;
                            Unsafe.Add(ref start8, bucketIndex) = previousSum8;
                            Unsafe.Add(ref start16, bucketIndex) = previousSum16;
                            Unsafe.Add(ref start24, bucketIndex) = previousSum24;
                            for (int i = 0; i < threadDispatcher.ThreadCount; ++i)
                            {
                                ref var worker = ref workers[i];
                                previousSum0 += worker.Buckets0[bucketIndex];
                                previousSum8 += worker.Buckets8[bucketIndex];
                                previousSum16 += worker.Buckets16[bucketIndex];
                                previousSum24 += worker.Buckets24[bucketIndex];
                            }
                        }
                        ReorderForByte(ref keys[0], ref keysScratch[0], ref values[0], ref valuesScratch[0], keyCount, ref bucketStarts0[0], 0);
                        ReorderForByte(ref keysScratch[0], ref keys[0], ref valuesScratch[0], ref values[0], keyCount, ref bucketStarts8[0], 8);
                        ReorderForByte(ref keys[0], ref keysScratch[0], ref values[0], ref valuesScratch[0], keyCount, ref bucketStarts16[0], 16);
                        ReorderForByte(ref keysScratch[0], ref keys[0], ref valuesScratch[0], ref values[0], keyCount, ref bucketStarts24[0], 24);
                        sortedKeys = keys;
                        sortedValues = values;
                    }
                    break;
                default:
                    Debug.Fail("This should be impossible- the switch cases should cover all possible values of bucket levels.");
                    sortedKeys = new Buffer<uint>();
                    sortedValues = new Buffer<TValue>();
                    break;

            }
            for (int workerIndex = 0; workerIndex < threadDispatcher.ThreadCount; ++workerIndex)
            {
                intPool.Return(ref workers[workerIndex].Buckets0);
                if (bucketSetCount >= 2) intPool.Return(ref workers[workerIndex].Buckets8);
                if (bucketSetCount >= 3) intPool.Return(ref workers[workerIndex].Buckets16);
                if (bucketSetCount >= 4) intPool.Return(ref workers[workerIndex].Buckets24);
            }
            intPool.Return(ref bucketStarts0);
            if (bucketSetCount >= 2) intPool.Return(ref bucketStarts8);
            if (bucketSetCount >= 3) intPool.Return(ref bucketStarts16);
            if (bucketSetCount >= 4) intPool.Return(ref bucketStarts24);
            bufferPool.SpecializeFor<Worker>().Return(ref workers);

        }

    }
}
