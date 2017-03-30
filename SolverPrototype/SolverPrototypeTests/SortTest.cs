﻿using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using SolverPrototype;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    public static class SortTest
    {

        struct Comparer : IComparerRef<int>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int Compare(ref int a, ref int b)
            {
                return a < b ? -1 : a > b ? 1 : 0;
            }
        }

        public static void Test()
        {
            void VerifySort(int[] keys)
            {
                for (int i = 1; i < keys.Length; ++i)
                {
                    Debug.Assert(keys[i] >= keys[i - 1]);
                }
            }
            const int elementCount = 262144;
            const int elementExclusiveUpperBound = 1 << 30;
            for (int iteration = 0; iteration < 16; ++iteration)
            {
                GC.Collect(3, GCCollectionMode.Forced, true);
                int[] keys = new int[elementCount];
                int[] indexMap = new int[elementCount];
                Random random = new Random(5);
                for (int i = 0; i < elementCount; ++i)
                {
                    indexMap[i] = i;
                    //keys[i] = i;
                    keys[i] = random.Next(elementExclusiveUpperBound);
                }
                var keys2 = new int[elementCount];
                var indexMap2 = new int[elementCount];
                var keys3 = new int[elementCount];
                var indexMap3 = new int[elementCount];
                var keys4 = new int[elementCount];
                var indexMap4 = new int[elementCount];
                Array.Copy(indexMap, indexMap2, elementCount);
                Array.Copy(keys, keys2, elementCount);
                Array.Copy(indexMap, indexMap3, elementCount);
                Array.Copy(keys, keys3, elementCount);
                Array.Copy(indexMap, indexMap4, elementCount);
                Array.Copy(keys, keys4, elementCount);


                var comparer = new Comparer();
                int swapCount = 0;
                Quicksort.Sort2(ref keys[0], ref indexMap[0], 0, 0, ref comparer); //prejit
                var timer = Stopwatch.StartNew();
                Quicksort.Sort2(ref keys[0], ref indexMap[0], 0, elementCount - 1, ref comparer);
                timer.Stop();
                VerifySort(keys);
                Console.WriteLine($"QSort2 time (ms): {timer.Elapsed.TotalSeconds * 1e3}, swapcount: {swapCount}");

                Array.Sort(keys2, indexMap2, 0, 1); //prejit
                timer.Restart();
                Array.Sort(keys2, indexMap2, 0, elementCount);
                timer.Stop();
                VerifySort(keys2);
                Console.WriteLine($"Array.Sort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                var keysScratch = new int[elementCount];
                var valuesScratch = new int[elementCount];
                var bucketCounts = new int[1024];
                Array.Clear(bucketCounts, 0, bucketCounts.Length);
                LSBRadixSort.SortU32(ref keys3[0], ref indexMap3[0], ref keysScratch[0], ref valuesScratch[0], ref bucketCounts[0], 1); //prejit
                timer.Restart();
                Array.Clear(bucketCounts, 0, bucketCounts.Length);
                LSBRadixSort.SortU32(ref keys3[0], ref indexMap3[0], ref keysScratch[0], ref valuesScratch[0], ref bucketCounts[0], elementCount);
                timer.Stop();
                VerifySort(keys3);
                Console.WriteLine($"LSBRadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                var originalIndices = new int[256];
                //MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], ref bucketCounts[0], ref originalIndices[0], 1, 24); //prejit
                MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], 1, SpanHelper.GetContainingPowerOf2(elementExclusiveUpperBound)); //prejit
                timer.Restart();
                //MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], ref bucketCounts[0], ref originalIndices[0], elementCount, 24);
                MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], elementCount, SpanHelper.GetContainingPowerOf2(elementExclusiveUpperBound));
                timer.Stop();
                VerifySort(keys4);
                Console.WriteLine($"MSBRadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");
            }

        }
    }
}
