using SolverPrototype;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    static class SuballocationTests
    {
        private static object debug;

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
                const int anchorSize = 128;
                for (int j = 0; j < 1000; ++j)
                {
                    var removeProbability = 0.5f + 0.5f * (list.Count - anchorSize) / anchorSize;
                    var p = random.NextDouble();
                    if (p < removeProbability)
                    {
                        Debug.Assert(list.Count > 0);
                        //Note that adds can invalidate the start.
                        ref var start = ref bufferPool.GetStart<int>(ref list.Region);
                        if (p < removeProbability * 0.5)
                        {
                            //Remove an element that is actually present.
                            var toRemoveIndex = random.Next(list.Count);
                            var toRemove = Unsafe.Add(ref start, toRemoveIndex);
                            var removed = SuballocatedList.FastRemove<int, TestComparer>(bufferPool, ref list, toRemove);
                            Debug.Assert(removed, "If we selected an element from the list, it should be removable.");
                        }
                        else
                        {
                            var toRemove = -(1 + random.Next(16));
                            var removed = SuballocatedList.FastRemove<int, TestComparer>(bufferPool, ref list, toRemove);
                            Debug.Assert(!removed, "Shouldn't be able to remove things that were never added!");
                        }
                    }
                    else
                    {
                        var toAdd = random.Next(256);
                        SuballocatedList.Add(bufferPool, ref list, toAdd);
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
