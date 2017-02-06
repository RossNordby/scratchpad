using SolverPrototype;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    static class SuballocationTests
    {
        public struct TestComparer : IEqualityComparerRef<int>, IEqualityComparer<int>
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Equals(ref int x, ref int y)
            {
                return x == y;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Equals(int x, int y)
            {
                return x == y;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetHashCode(ref int i)
            {
                return i;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int GetHashCode(int i)
            {
                return i;
            }
        }
        public static void Test()
        {
            Pow2Allocator allocator = new Pow2Allocator(16, 1024);
            var bufferPool = new SuballocatedBufferPool(i => i >= 3 && i <= 6 ? 1 << 20 : 0, allocator);

            var random = new Random(5);
            const int listCount = 1000;
            var lists = new SuballocatedList[listCount];
            for (int i = 0; i < 1000; ++i)
            {
                SuballocatedList.Create(bufferPool, 0 + random.Next(9), out lists[i]);
                ref var list = ref lists[i];
                const int targetSize = 128;
                for (int j = 0; j < 1000; ++j)
                {
                    var removeProbability = 0.5f + 0.5f * (list.Count - targetSize) / targetSize;
                    if (random.NextDouble() < removeProbability)
                    {
                        var toRemove = random.Next(16);
                        SuballocatedList.FastRemove<int, TestComparer>(bufferPool, ref list, toRemove);
                    }
                    else
                    {
                        SuballocatedList.Add(bufferPool, ref list, random.Next(16));
                    }
                }
            }
            for (int i = 0; i < listCount; ++i)
            {
                SuballocatedList.Dispose(bufferPool, ref lists[i]);
            }
        }
    }
}
