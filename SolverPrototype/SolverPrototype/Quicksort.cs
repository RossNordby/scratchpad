using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    public static class Quicksort
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

        public static void MappedSort<T, TComparer>(ref T keys, ref int indexMap, int l, int r, ref TComparer comparer) where TComparer : IComparerRef<T>
        {
            if (r - l <= 30)
            {
                //The area to address is very small. Use insertion sort.
                for (int i = l + 1; i <= r; ++i)
                {
                    var originalIndexMap = Unsafe.Add(ref indexMap, i);
                    ref var original = ref Unsafe.Add(ref keys, originalIndexMap);
                    int compareIndex;
                    for (compareIndex = i - 1; compareIndex >= l; --compareIndex)
                    {
                        if (comparer.Compare(ref original, ref Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, compareIndex))) < 0)
                        {
                            //Move the index up.
                            Unsafe.Add(ref indexMap, compareIndex + 1) = Unsafe.Add(ref indexMap, compareIndex);
                        }
                        else
                            break;
                    }
                    var targetIndex = compareIndex + 1;
                    if (targetIndex != i)
                    {
                        //Move the original index down.
                        Unsafe.Add(ref indexMap, targetIndex) = originalIndexMap;
                    }
                }
            }
            else
            {
                //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
                var first = Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, l));
                int middleIndex = (l + r) / 2;
                var middle = Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, middleIndex));
                var last = Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, r));
                int pivotIndex;
                T pivot;
                if (comparer.Compare(ref first, ref middle) <= 0 && comparer.Compare(ref first, ref last) <= 0)
                {
                    //first is lowest.
                    if (comparer.Compare(ref middle, ref last) <= 0)
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else if (comparer.Compare(ref middle, ref last) <= 0)
                {
                    //middle is lowest.
                    if (comparer.Compare(ref first, ref last) <= 0)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else
                {
                    //last is lowest.
                    if (comparer.Compare(ref first, ref middle) <= 0)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                }

                //Put the pivot into the last slot.
                Swap(ref Unsafe.Add(ref indexMap, pivotIndex), ref Unsafe.Add(ref indexMap, r));

                //Use bentley-mcilroy 3-way partitioning scheme to avoid performance drops in corner cases.
                int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
                int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
                int p = l - 1;
                int q = r;
                if (r <= l)
                    return;
                while (true)
                {
                    //Claim the chunk of the list which is partitioned on the left and right sides.
                    while (true)
                    {
                        ++i;
                        if (comparer.Compare(ref Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, i)), ref pivot) >= 0)
                            break;
                    }
                    while (true)
                    {
                        --j;
                        if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, j))) >= 0 || j == l)
                            break;

                    }
                    //If the claims have met, then the partition is complete.
                    if (i >= j)
                    {
                        break;
                    }
                    //By the claiming above and because we did not yet break out of the loop,
                    //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
                    //So swap them.
                    Swap(ref Unsafe.Add(ref indexMap, i), ref Unsafe.Add(ref indexMap, j));
                    if (comparer.Compare(ref Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, i)), ref pivot) == 0)
                    {
                        p++;
                        Swap(ref Unsafe.Add(ref indexMap, p), ref Unsafe.Add(ref indexMap, i));
                    }
                    if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, Unsafe.Add(ref indexMap, j))) == 0)
                    {
                        q--;
                        Swap(ref Unsafe.Add(ref indexMap, j), ref Unsafe.Add(ref indexMap, q));
                    }
                }
                //The pivot at r has not been swapped.
                //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
                //So, swap the pivot and the first element of the greater-than-pivot side to guarantee sorting.
                Swap(ref Unsafe.Add(ref indexMap, i), ref Unsafe.Add(ref indexMap, r));
                j = i - 1;
                i = i + 1;
                for (int k = l; k < p; k++, j--)
                {
                    Swap(ref Unsafe.Add(ref indexMap, k), ref Unsafe.Add(ref indexMap, j));
                }
                for (int k = r - 1; k > q; k--, i++)
                {
                    Swap(ref Unsafe.Add(ref indexMap, i), ref Unsafe.Add(ref indexMap, k));
                }
                MappedSort(ref keys, ref indexMap, l, j, ref comparer);
                MappedSort(ref keys, ref indexMap, i, r, ref comparer);
            }
        }


        public static void Sort<TKey, TValue, TComparer>(ref TKey keys, ref TValue values, int l, int r, ref TComparer comparer) where TComparer : IComparerRef<TKey>
        {
            if (r - l <= 30)
            {
                //The area to address is very small. Use insertion sort.
                for (int i = l + 1; i <= r; ++i)
                {
                    var originalKey = Unsafe.Add(ref keys, i);
                    var originalValue = Unsafe.Add(ref values, i);
                    int compareIndex;
                    for (compareIndex = i - 1; compareIndex >= l; --compareIndex)
                    {
                        if (comparer.Compare(ref originalKey, ref Unsafe.Add(ref keys, compareIndex)) < 0)
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
            }
            else
            {
                //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
                var first = Unsafe.Add(ref keys, l);
                int middleIndex = (l + r) / 2;
                var middle = Unsafe.Add(ref keys, middleIndex);
                var last = Unsafe.Add(ref keys, r);
                int pivotIndex;
                TKey pivot;
                if (comparer.Compare(ref first, ref middle) <= 0 && comparer.Compare(ref first, ref last) <= 0)
                {
                    //first is lowest.
                    if (comparer.Compare(ref middle, ref last) <= 0)
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else if (comparer.Compare(ref middle, ref last) <= 0)
                {
                    //middle is lowest.
                    if (comparer.Compare(ref first, ref last) <= 0)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else
                {
                    //last is lowest.
                    if (comparer.Compare(ref first, ref middle) <= 0)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                }

                //Put the pivot into the last slot.
                Swap(ref keys, ref values, pivotIndex, r);

                //Use bentley-mcilroy 3-way partitioning scheme to avoid performance drops in corner cases.
                int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
                int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
                int p = l - 1;
                int q = r;
                if (r <= l)
                    return;
                while (true)
                {
                    //Claim the chunk of the list which is partitioned on the left and right sides.
                    while (true)
                    {
                        ++i;
                        if (comparer.Compare(ref Unsafe.Add(ref keys, i), ref pivot) >= 0)
                            break;
                    }
                    while (true)
                    {
                        --j;
                        if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, j)) >= 0 || j == l)
                            break;

                    }
                    //If the claims have met, then the partition is complete.
                    if (i >= j)
                    {
                        break;
                    }
                    //By the claiming above and because we did not yet break out of the loop,
                    //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
                    //So swap them.
                    Swap(ref keys, ref values, i, j);
                    if (comparer.Compare(ref Unsafe.Add(ref keys, i), ref pivot) == 0)
                    {
                        p++;
                        Swap(ref keys, ref values, p, i);
                    }
                    if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, j)) == 0)
                    {
                        q--;
                        Swap(ref keys, ref values, j, q);
                    }
                }
                //The pivot at r has not been swapped.
                //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
                //So, swap the pivot and the first element of the greater-than-pivot side to guarantee sorting.
                Swap(ref keys, ref values, i, r);
                j = i - 1;
                i = i + 1;
                for (int k = l; k < p; k++, j--)
                {
                    Swap(ref keys, ref values, k, j);
                }
                for (int k = r - 1; k > q; k--, i++)
                {
                    Swap(ref keys, ref values, i, k);
                }
                Sort(ref keys, ref values, l, j, ref comparer);
                Sort(ref keys, ref values, i, r, ref comparer);
            }
        }

        public static void Sort2<TKey, TValue, TComparer>(ref TKey keys, ref TValue values, int l, int r, ref TComparer comparer) where TComparer : IComparerRef<TKey>
        {
            if (r - l <= 32)
            {
                //The area to address is very small. Use insertion sort.
                for (int i = l + 1; i <= r; ++i)
                {
                    var originalKey = Unsafe.Add(ref keys, i);
                    var originalValue = Unsafe.Add(ref values, i);
                    int compareIndex;
                    for (compareIndex = i - 1; compareIndex >= l; --compareIndex)
                    {
                        if (comparer.Compare(ref originalKey, ref Unsafe.Add(ref keys, compareIndex)) < 0)
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
            }
            else
            {
                //Use MO3 to find a pivot to compensate for the common already-sorted case and to slightly improve worst-case behavior.
                var first = Unsafe.Add(ref keys, l);
                int middleIndex = (l + r) / 2;
                var middle = Unsafe.Add(ref keys, middleIndex);
                var last = Unsafe.Add(ref keys, r);
                int pivotIndex;
                TKey pivot;
                if (comparer.Compare(ref first, ref middle) <= 0 && comparer.Compare(ref first, ref last) <= 0)
                {
                    //first is lowest.
                    if (comparer.Compare(ref middle, ref last) <= 0)
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else if (comparer.Compare(ref middle, ref last) <= 0)
                {
                    //middle is lowest.
                    if (comparer.Compare(ref first, ref last) <= 0)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = r;
                        pivot = last;
                    }
                }
                else
                {
                    //last is lowest.
                    if (comparer.Compare(ref first, ref middle) <= 0)
                    {
                        pivotIndex = l;
                        pivot = first;
                    }
                    else
                    {
                        pivotIndex = middleIndex;
                        pivot = middle;
                    }
                }

                //Put the pivot into the last slot.
                Swap(ref keys, ref values, pivotIndex, r);
                
                int i = l - 1; //Start one below the partitioning area, because don't know if the first index is actually claimable.
                int j = r; //The last element of the partition holds the pivot, which is excluded from the swapping process. Same logic as for i.
                while (true)
                {
                    //Claim the chunk of the list which is partitioned on the left and right sides.
                    while (true)
                    {
                        ++i;
                        if (comparer.Compare(ref Unsafe.Add(ref keys, i), ref pivot) >= 0)
                            break;
                    }
                    while (true)
                    {
                        --j;
                        if (comparer.Compare(ref pivot, ref Unsafe.Add(ref keys, j)) >= 0 || j == l)
                            break;

                    }
                    //If the claims have met, then the partition is complete.
                    if (i >= j)
                    {
                        break;
                    }
                    //By the claiming above and because we did not yet break out of the loop,
                    //the value associated with i is >= the pivot, and the value associated with j is <= the pivot.
                    //So swap them.
                    Swap(ref keys, ref values, i, j);
                }
                //The pivot at r has not been swapped.
                //Since the loop has terminated, we know that i has reached the the '>=pivot' side of the partition.
                //So, swap the pivot and the first element of the greater-than-pivot side to guarantee sorting.
                Swap(ref keys, ref values, i, r);
                j = i - 1; //Sort's parameters take an inclusive bound, so push j back.
                i = i + 1; //The pivot takes i's spot, and the pivot should not be included in sorting, so push i up.
                Sort2(ref keys, ref values, l, j, ref comparer);
                Sort2(ref keys, ref values, i, r, ref comparer);
            }
        }
    }
}
