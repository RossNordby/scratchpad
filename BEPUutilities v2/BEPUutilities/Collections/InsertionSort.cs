using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BEPUutilities2.Collections
{
    public static class InsertionSort
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Sort<TKey, TValue, TKeySpan, TValueSpan, TComparer>
            (ref TKeySpan keys, ref TValueSpan values, int start, int inclusiveEnd, ref TComparer comparer)
           where TComparer : IComparerRef<TKey>
           where TKeySpan : ISpan<TKey>
           where TValueSpan : ISpan<TValue>
        {
            Debug.Assert(keys.Length <= values.Length);
            Debug.Assert(start >= 0 && start < keys.Length);
            Debug.Assert(inclusiveEnd < keys.Length); //We don't bother checking if inclusiveEnd is >= start; a zero length region will be caught by the loop condition.

            for (int i = start + 1; i <= inclusiveEnd; ++i)
            {
                var originalKey = keys[i];
                var originalValue = values[i];
                int compareIndex;
                for (compareIndex = i - 1; compareIndex >= start; --compareIndex)
                {
                    if (comparer.Compare(ref originalKey, ref keys[compareIndex]) < 0)
                    {
                        //Move the element up one slot.
                        var upperSlotIndex = compareIndex + 1;
                        keys[upperSlotIndex] = keys[compareIndex];
                        values[upperSlotIndex] = values[compareIndex];
                    }
                    else
                        break;
                }
                var targetIndex = compareIndex + 1;
                if (targetIndex != i)
                {
                    //Move the original index down.
                    keys[targetIndex] = originalKey;
                    values[targetIndex] = originalValue;
                }
            }
        }
    }
}
