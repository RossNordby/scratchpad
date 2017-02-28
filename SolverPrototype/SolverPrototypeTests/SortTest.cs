using SolverPrototype;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    public static class SortTest
    {
        struct Comparer : IComparerRef<int>
        {
            public int Compare(ref int a, ref int b)
            {
                return a < b ? -1 : a > b ? 1 : 0;
            }
        }

        public static void Test()
        {
            const int elementCount = 262144;
            int[] keys = new int[elementCount];
            int[] indexMap = new int[elementCount];
            Random random = new Random(5);
            for (int i =0; i < elementCount; ++i)
            {
                indexMap[i] = i;
                keys[i] = random.Next();
            }
            var indexMap2 = new int[elementCount];
            Array.Copy(indexMap, indexMap2, elementCount);

            var comparer = new Comparer();
            Quicksort.MappedSort(ref keys[0], ref indexMap[0], 0, 0, ref comparer); //prejit
            var timer = Stopwatch.StartNew();
            Quicksort.MappedSort(ref keys[0], ref indexMap[0], 0, elementCount - 1, ref comparer);

            timer.Stop();
            Console.WriteLine($"QSort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");
            int[] result = new int[elementCount];
            for (int i = 0; i < elementCount; ++i)
            {
                result[i] = keys[indexMap[i]];
            }


            Array.Sort(keys, indexMap2, 0, 1); //prejit
            timer.Restart();
            Array.Sort(keys, indexMap2, 0, elementCount);
            timer.Stop();
            Console.WriteLine($"Array.Sort time (ms): {timer.Elapsed.TotalSeconds * 1e3}");
        }
    }
}
