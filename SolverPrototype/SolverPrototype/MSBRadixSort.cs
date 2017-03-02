using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{

    public static class MSBRadixSort
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Swap<T>(ref T a, ref T b)
        {
            var temp = a;
            a = b;
            b = temp;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Swap<TKey, TValues>(ref TKey keys, ref TValues values, int a, int b)
        {
            Swap(ref Unsafe.Add(ref keys, a), ref Unsafe.Add(ref keys, b));
            Swap(ref Unsafe.Add(ref values, a), ref Unsafe.Add(ref values, b));
        }

        public static void SortU32<T>(ref int keys, ref T values, ref int bucketCounts, ref int bucketOriginalStartIndices, int keyCount, int shift)
        {
            if (keyCount < 30)
            {
                //There aren't many keys remaining. Use insertion sort.
                for (int i = 1; i < keyCount; ++i)
                {
                    var originalKey = Unsafe.Add(ref keys, i);
                    var originalValue = Unsafe.Add(ref values, i);
                    int compareIndex;
                    for (compareIndex = i - 1; compareIndex >= 0; --compareIndex)
                    {
                        if (originalKey < Unsafe.Add(ref keys, compareIndex))
                        {
                            //Move the element up one slot.
                            var upperSlotIndex = compareIndex + 1;
                            Unsafe.Add(ref keys, upperSlotIndex) = Unsafe.Add(ref keys, compareIndex);
                            Unsafe.Add(ref values, upperSlotIndex) = Unsafe.Add(ref values, compareIndex);
                        }
                        else
                            break;
                    }
                    var targetIndex = compareIndex + 1;
                    if (targetIndex != i)
                    {
                        //Move the original index down.
                        Unsafe.Add(ref keys, targetIndex) = originalKey;
                        Unsafe.Add(ref values, targetIndex) = originalValue;
                    }
                }
                return;
            }
            const int bucketCount = 256;
            //Each section of the bucketCounts cover 256 slots, representing all possible values for a byte.
            //The bucketCounts array passed into the root function must contain enough space to hold every level of the recursion, which is at maximum 1024 entries for 32 bit sorts.
            for (int i = 0; i < keyCount; ++i)
            {
                var key = Unsafe.Add(ref keys, i);
                ++Unsafe.Add(ref bucketCounts, key & 0xFF);
            }

            //Convert the bucket counts to partial sums.
            int sum = 0;
            for (int i = 0; i < bucketCount; ++i)
            {
                var previousSum = sum;
                ref var bucketSlotCount = ref Unsafe.Add(ref bucketCounts, i);
                sum += bucketSlotCount;
                //We store the partial sum into both the bucketCounts array and the originalStartIndices array.
                Unsafe.Add(ref bucketOriginalStartIndices, i) = previousSum;
                bucketSlotCount = previousSum;
            }

            //Note that the bucketCounts array is now really a startIndices array. We'll increment the bucketCounts version of the array as we perform swaps.
            //Walk through each bucket's region in the array, and walk through each element within each bucket. For each element, push it to the target bucket.
            //Rather than writing the displaced element into the local bucket, check where it should go. Repeat until an element that fits the current bucket is found.
            //Only upon finding the fitting element should anything be written to the local bucket slot. Note that the counting phase guarantees that an element will be found.
            //Every write into a target bucket should increment that bucket's start index.
            //When the number of elements visited reaches the size of the bucket, move on to the next bucket and start over.
            for (int i = 0; i < bucketCount; ++i)
            {
                ref var bucketStartIndex = ref Unsafe.Add(ref bucketCounts, i);
                //Note the use of the original start index. If it kept going beyond the original start, it will walk into an already sorted region.
                int nextStartIndex = i == bucketCount - 1 ? keyCount : Unsafe.Add(ref bucketOriginalStartIndices, i + 1);
                while (bucketStartIndex < nextStartIndex)
                {
                    var localKey = Unsafe.Add(ref keys, bucketStartIndex);
                    while (true)
                    {
                        var targetBucketIndex = (localKey >> shift) & 0xFF;
                        if (targetBucketIndex == i)
                        {
                            //The local key belongs to the local bucket. We can stop the swaps.
                            //Put the local key into the local bucket.
                            Unsafe.Add(ref keys, bucketStartIndex) = localKey;
                            ++bucketStartIndex;
                            break;
                        }
                        ref var targetBucketStartIndex = ref Unsafe.Add(ref bucketCounts, targetBucketIndex);
                        ref var targetSlot = ref Unsafe.Add(ref keys, targetBucketStartIndex);
                        var swappedKey = targetSlot;
                        targetSlot = localKey;
                        localKey = swappedKey;
                        ++targetBucketStartIndex;
                    }
                }
            }

            //Now every bucket contains the elements that belong to it. There may still be sorting to do.
            if (shift < 24)
            {
                //There is at least one more level of sorting potentially required.
                var newShift = shift + 8;
                for (int i = 0; i < bucketCount; ++i)
                {
                    SortU32(ref keys, ref values, ref Unsafe.Add(ref bucketCounts, bucketCount), ref bucketOriginalStartIndices, keyCount, newShift);
                }
            }
        }
    }
}
