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

        public struct TestPredicate : IPredicate<int>, IPredicateRef<int>
        {
            public int ToCompare;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Test(ref int x)
            {
                return x == ToCompare;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool Test(int x)
            {
                return x == ToCompare;
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
                            var predicate = new TestPredicate { ToCompare = toRemove };
                            var removed = SuballocatedList.FastRemove<int, TestPredicate>(bufferPool, ref list, ref predicate);
                            Debug.Assert(removed, "If we selected an element from the list, it should be removable.");
                        }
                        else
                        {
                            var toRemove = -(1 + random.Next(16));
                            var predicate = new TestPredicate { ToCompare = toRemove };
                            var removed = SuballocatedList.FastRemove<int, TestPredicate>(bufferPool, ref list, ref predicate);
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
