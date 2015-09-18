using System.Runtime.CompilerServices;

namespace SIMDPrototyping.Trees.SingleArray
{
    unsafe struct PriorityQueue
    {
        public struct Entry
        {
            public int Id;
            public float Cost;
        }
        public Entry* Entries;
        public int Count;

        public PriorityQueue(Entry* entries)
        {
            Entries = entries;
            Count = 0;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Insert(int id, float cost)
        {
            int index = Count++;

            //Sift up.
            while (index > 0)
            {
                var parentIndex = (index - 1) >> 1;
                var parent = Entries + parentIndex;
                if (parent->Cost < cost)
                {
                    //Pull the parent down.
                    Entries[index] = *parent;
                    index = parentIndex;
                }
                else
                {
                    //Found the insertion spot.
                    break;
                }
            }
            var entry = Entries + index;
            entry->Id = id;
            entry->Cost = cost;


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PopMax(out Entry entry)
        {
            entry = Entries[0];
            --Count;
            var cost = Entries[Count].Cost;

            //Pull the elements up to fill in the gap.
            int index = 0;
            while (true)
            {
                var childIndexA = (index << 1) + 1;
                var childIndexB = (index << 1) + 2;
                if (childIndexB < Count)
                {
                    //Both children are available.
                    //Try swapping with the largest one.
                    var childA = Entries + childIndexA;
                    var childB = Entries + childIndexB;
                    if (childA->Cost > childB->Cost)
                    {
                        if (cost > childA->Cost)
                        {
                            break;
                        }
                        Entries[index] = Entries[childIndexA];
                        index = childIndexA;
                    }
                    else
                    {
                        if (cost > childB->Cost)
                        {
                            break;
                        }
                        Entries[index] = Entries[childIndexB];
                        index = childIndexB;
                    }
                }
                else if (childIndexA < Count)
                {
                    //Only one child was available.
                    var childA = Entries + childIndexA;
                    if (cost > childA->Cost)
                    {
                        break;
                    }
                    Entries[index] = Entries[childIndexA];
                    index = childIndexA;
                }
                else
                {
                    //The children were beyond the heap.
                    break;
                }
            }
            //Move the last entry into position.
            Entries[index] = Entries[Count];

        }

    }
}
