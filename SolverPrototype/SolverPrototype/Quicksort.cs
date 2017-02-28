using System;
using System.Collections.Generic;
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
        static bool InsertionIteration<T, TComparer>(ref T keys, ref int indexMap, int l, ref int previousIndex, ref int index, ref TComparer comparer) where TComparer : IComparerRef<T>
        {
            ref var previous = ref Unsafe.Add(ref indexMap, previousIndex);
            ref var current = ref Unsafe.Add(ref indexMap, index);
            if (comparer.Compare(ref Unsafe.Add(ref keys, current), ref Unsafe.Add(ref keys, previous)) == -1)
            {
                Swap(ref previous, ref current);
                if (previousIndex == l)
                    return false;
                index = previousIndex;
                --previousIndex;
                return true;
            }
            return false;


        }
        public static void MappedSort<T, TComparer>(ref T keys, ref int indexMap, int l, int r, ref TComparer comparer) where TComparer : IComparerRef<T>
        {
            if (r - l <= 30)
            {
                //The area to address is very small. Use insertion sort.
                for (int i = l + 1; i <= r; ++i)
                {
                    //TODO: This could be better. Performs two way swaps at every step rather than 
                    //simply moving everything up one and then later moving the displaced element down.
                    var index = i;
                    var previousIndex = index - 1;
                    while (InsertionIteration(ref keys, ref indexMap, l, ref previousIndex, ref index, ref comparer)) ;
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
    }
}
