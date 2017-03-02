using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

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
            const int elementCount = 16384;
            int[] keys = new int[elementCount];
            int[] indexMap = new int[elementCount];
            Random random = new Random(5);
            for (int i =0; i < elementCount; ++i)
            {
                indexMap[i] = i;
                keys[i] = random.Next();
            }
            var keys2 = new int[elementCount];
            var indexMap2 = new int[elementCount];
            var keys3 = new int[elementCount];
            var indexMap3 = new int[elementCount];
            Array.Copy(indexMap, indexMap2, elementCount);
            Array.Copy(keys, keys2, elementCount);
            Array.Copy(indexMap, indexMap3, elementCount);
            Array.Copy(keys, keys3, elementCount);


            var comparer = new Comparer();
            Quicksort.Sort(ref keys[0], ref indexMap[0], 0, 0, ref comparer); //prejit
            var timer = Stopwatch.StartNew();
            Quicksort.Sort(ref keys[0], ref indexMap[0], 0, elementCount - 1, ref comparer);
            timer.Stop();
            Console.WriteLine($"QSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");
            int[] mappedResult = new int[elementCount];
            for (int i = 0; i < elementCount; ++i)
            {
                mappedResult[i] = keys[indexMap[i]];
            }


            var keysScratch = new int[elementCount];
            var valuesScratch = new int[elementCount];
            var bucketCounts = new int[1024];
            Array.Clear(bucketCounts, 0, bucketCounts.Length);
            LSBRadixSort.SortU32(ref keys3[0], ref indexMap3[0], ref keysScratch[0], ref valuesScratch[0], ref bucketCounts[0], 1); //prejit
            timer.Restart();
            Array.Clear(bucketCounts, 0, bucketCounts.Length);
            LSBRadixSort.SortU32(ref keys3[0], ref indexMap3[0], ref keysScratch[0], ref valuesScratch[0], ref bucketCounts[0], elementCount); //prejit
            timer.Stop();
            Console.WriteLine($"RadixSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");


            Array.Sort(keys2, indexMap2, 0, 1); //prejit
            timer.Restart();
            Array.Sort(keys2, indexMap2, 0, elementCount);
            timer.Stop();
            Console.WriteLine($"Array.Sort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");
        }
    }
}
