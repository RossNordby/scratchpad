using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
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
            void VerifySort<TSpan>(TSpan keys) where TSpan : ISpan<int>
            {
                for (int i = 1; i < keys.Length; ++i)
                {
                    Debug.Assert(keys[i] >= keys[i - 1]);
                }
            }
            const int elementCount = 4096;
            const int elementExclusiveUpperBound = 32768;// 1 << 5;
            for (int iteration = 0; iteration < 4; ++iteration)
            {
                GC.Collect(3, GCCollectionMode.Forced, true);
                var keys = new Array<int>(new int[elementCount]);
                var indexMap = new Array<int>(new int[elementCount]);
                Random random = new Random(5);
                for (int i = 0; i < elementCount; ++i)
                {
                    indexMap[i] = i;
                    //keys[i] = i / (elementCount / elementExclusiveUpperBound);
                    //keys[i] = i % elementExclusiveUpperBound;
                    //keys[i] = i;
                    keys[i] = random.Next(elementExclusiveUpperBound);
                }
                var keys2 = new Array<int>(new int[elementCount]);
                var indexMap2 = new Array<int>(new int[elementCount]);
                var keys3 = new Array<int>(new int[elementCount]);
                var indexMap3 = new Array<int>(new int[elementCount]);
                var keys4 = new Array<int>(new int[elementCount]);
                var indexMap4 = new Array<int>(new int[elementCount]);
                keys.CopyTo(0, ref keys2, 0, elementCount);
                keys.CopyTo(0, ref keys3, 0, elementCount);
                keys.CopyTo(0, ref keys4, 0, elementCount);
                indexMap.CopyTo(0, ref indexMap2, 0, elementCount);
                indexMap.CopyTo(0, ref indexMap3, 0, elementCount);
                indexMap.CopyTo(0, ref indexMap4, 0, elementCount);

                var timer = Stopwatch.StartNew();




                var keysScratch = new int[elementCount];
                var valuesScratch = new int[elementCount];
                var bucketCounts = new int[1024];
                for (int t = 0; t < 16; ++t)
                {
                    //keys2.CopyTo(0, ref keys, 0, elementCount);
                    //unsafe
                    //{
                    //    var comparer = new Comparer();
                    //    fixed (int* keysPointer = keys.Memory)
                    //    fixed (int* valuesPointer = indexMap.Memory)
                    //    {
                    //        var keysBuffer = new Buffer<int>(keysPointer, keys.Length);
                    //        var valuesBuffer = new Buffer<int>(valuesPointer, indexMap.Length);
                    //        timer.Restart();
                    //        QuickSort.Sort(ref keysBuffer[0], ref valuesBuffer[0], 0, elementCount - 1, ref comparer);
                    //    }
                    //}
                    ////QuickSort.Sort2(ref keys[0], ref indexMap[0], 0, elementCount - 1, ref comparer);
                    //timer.Stop();
                    //VerifySort(keys);
                    //Console.WriteLine($"QuickSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                    //keys.CopyTo(0, ref keys2, 0, elementCount);
                    //timer.Restart();
                    //Array.Sort(keys2.Memory, indexMap2.Memory, 0, elementCount);
                    //timer.Stop();
                    //VerifySort(keys2);
                    //Console.WriteLine($"Array.Sort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                    //keys.CopyTo(0, ref keys3, 0, elementCount);
                    //timer.Restart();
                    //Array.Clear(bucketCounts, 0, bucketCounts.Length);
                    //LSBRadixSort.SortU16(ref keys3[0], ref indexMap3[0], ref keysScratch[0], ref valuesScratch[0], ref bucketCounts[0], elementCount);
                    //timer.Stop();
                    //VerifySort(keys3);
                    //Console.WriteLine($"{t} LSBRadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                    //keys.CopyTo(0, ref keys4, 0, elementCount);
                    //var originalIndices = new int[256];
                    //timer.Restart();
                    ////MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], ref bucketCounts[0], ref originalIndices[0], elementCount, 24);
                    //MSBRadixSort.SortU32(ref keys4[0], ref indexMap4[0], elementCount, SpanHelper.GetContainingPowerOf2(elementExclusiveUpperBound));
                    //timer.Stop();
                    //VerifySort(keys4);
                    //Console.WriteLine($"{t} MSBRadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");

                    keys.CopyTo(0, ref keys3, 0, elementCount);
                    timer.Restart();
                    var sorter = new ParallelLSBRadixSort();
                    unsafe
                    {
                        fixed (int* keys3Pointer = keys3.Memory)
                        fixed (int* indexMap3Pointer = indexMap3.Memory)
                        fixed (int* keysScratchPointer = keysScratch)
                        fixed (int* valuesScratchPointer = valuesScratch)
                        {
                            var keys3Buffer = new Buffer<uint>(keys3Pointer, keys3.Length);
                            var indexMap3Buffer = new Buffer<int>(keys3Pointer, keys3.Length);
                            var keysScratchBuffer = new Buffer<uint>(keys3Pointer, keys3.Length);
                            var valuesScratchBuffer = new Buffer<int>(keys3Pointer, keys3.Length);
                            sorter.Sort(ref keys3, ref indexMap3, ref keysScratch, 0, keys3.Length, elementExclusiveUpperBound, ref valuesScratch, 
                                bufferPool, threadDispatcher, out var sortedKeys, out var sortedValues);

                            VerifySort(sortedKeys);
                        }
                    }
                    timer.Stop();
                    Console.WriteLine($"{t} LSBRadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");
                }
            }

        }
    }
}
