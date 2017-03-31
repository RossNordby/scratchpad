using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilities2.Collections
{
    public static class LSBRadixSort
    {
        //TODO: If the jit ever managed to handle ISpan indexers optimally, we could use a much more natural ISpan-based implementation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ReorderForByte<T>(ref int sourceKeys, ref int targetKeys, ref T sourceValues, ref T targetValues, int keyCount, ref int indices, int shift)
        {
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref sourceKeys, i);
                ref var index = ref Unsafe.Add(ref indices, (key >> shift) & 0xFF);
                Unsafe.Add(ref targetKeys, index) = key;
                Unsafe.Add(ref targetValues, index) = Unsafe.Add(ref sourceValues, i);
                //Bump the index up to compensate for the new element.
                ++index;
            }
        }

        public static void SortU32<T>(ref int keys, ref T values, ref int keysScratch, ref T valuesScratch, ref int bucketCounts, int keyCount)
        {
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            ref var byte1Counts = ref Unsafe.Add(ref bucketCounts, 256);
            ref var byte2Counts = ref Unsafe.Add(ref bucketCounts, 512);
            ref var byte3Counts = ref Unsafe.Add(ref bucketCounts, 768);
            
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref keys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
                ++Unsafe.Add(ref byte1Counts, (key >> 8) & 0xFF);
                ++Unsafe.Add(ref byte2Counts, (key >> 16) & 0xFF);
                ++Unsafe.Add(ref byte3Counts, key >> 24);
            }

            //Convert the bucket counts to partial sums.
            int sum0 = 0;
            int sum1 = 0;
            int sum2 = 0;
            int sum3 = 0;
            for (int i = 0; i < 256; ++i)
            {
                var previousSum0 = sum0;
                var previousSum1 = sum1;
                var previousSum2 = sum2;
                var previousSum3 = sum3;
                ref var byte0 = ref Unsafe.Add(ref bucketCounts, i);
                ref var byte1 = ref Unsafe.Add(ref byte1Counts, i);
                ref var byte2 = ref Unsafe.Add(ref byte2Counts, i);
                ref var byte3 = ref Unsafe.Add(ref byte3Counts, i);
                sum0 += byte0;
                sum1 += byte1;
                sum2 += byte2;
                sum3 += byte3;
                byte0 = previousSum0;
                byte1 = previousSum1;
                byte2 = previousSum2;
                byte3 = previousSum3;
            }
            
            ReorderForByte(ref keys, ref keysScratch, ref values, ref valuesScratch, keyCount, ref bucketCounts, 0);
            ReorderForByte(ref keysScratch, ref keys, ref valuesScratch, ref values, keyCount, ref byte1Counts, 8);
            ReorderForByte(ref keys, ref keysScratch, ref values, ref valuesScratch, keyCount, ref byte2Counts, 16);
            ReorderForByte(ref keysScratch, ref keys, ref valuesScratch, ref values, keyCount, ref byte3Counts, 24);
        }

        public static void SortU24<T>(ref int inputKeys, ref T inputValues, ref int outputKeys, ref T outputValues, ref int bucketCounts, int keyCount)
        {
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            ref var byte1Counts = ref Unsafe.Add(ref bucketCounts, 256);
            ref var byte2Counts = ref Unsafe.Add(ref bucketCounts, 512);

            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref inputKeys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
                ++Unsafe.Add(ref byte1Counts, (key >> 8) & 0xFF);
                ++Unsafe.Add(ref byte2Counts, (key >> 16) & 0xFF);
            }

            //Convert the bucket counts to partial sums.
            int sum0 = 0;
            int sum1 = 0;
            int sum2 = 0;
            for (int i = 0; i < 256; ++i)
            {
                var previousSum0 = sum0;
                var previousSum1 = sum1;
                var previousSum2 = sum2;
                ref var byte0 = ref Unsafe.Add(ref bucketCounts, i);
                ref var byte1 = ref Unsafe.Add(ref byte1Counts, i);
                ref var byte2 = ref Unsafe.Add(ref byte2Counts, i);
                sum0 += byte0;
                sum1 += byte1;
                sum2 += byte2;
                byte0 = previousSum0;
                byte1 = previousSum1;
                byte2 = previousSum2;
            }

            ReorderForByte(ref inputKeys, ref outputKeys, ref inputValues, ref outputValues, keyCount, ref bucketCounts, 0);
            ReorderForByte(ref outputKeys, ref inputKeys, ref outputValues, ref inputValues, keyCount, ref byte1Counts, 8);
            ReorderForByte(ref inputKeys, ref outputKeys, ref inputValues, ref outputValues, keyCount, ref byte2Counts, 16);
        }

        public static void SortU16<T>(ref int keys, ref T values, ref int keysScratch, ref T valuesScratch, ref int bucketCounts, int keyCount)
        {
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            ref var byte1Counts = ref Unsafe.Add(ref bucketCounts, 256);

            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref keys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
                ++Unsafe.Add(ref byte1Counts, (key >> 8) & 0xFF);
            }

            //Convert the bucket counts to partial sums.
            int sum0 = 0;
            int sum1 = 0;
            int sum2 = 0;
            int sum3 = 0;
            for (int i = 0; i < 256; ++i)
            {
                var previousSum0 = sum0;
                var previousSum1 = sum1;
                var previousSum2 = sum2;
                var previousSum3 = sum3;
                ref var byte0 = ref Unsafe.Add(ref bucketCounts, i);
                ref var byte1 = ref Unsafe.Add(ref byte1Counts, i);
                sum0 += byte0;
                sum1 += byte1;
                byte0 = previousSum0;
                byte1 = previousSum1;
            }

            ReorderForByte(ref keys, ref keysScratch, ref values, ref valuesScratch, keyCount, ref bucketCounts, 0);
            ReorderForByte(ref keysScratch, ref keys, ref valuesScratch, ref values, keyCount, ref byte1Counts, 8);
        }

    }
}
