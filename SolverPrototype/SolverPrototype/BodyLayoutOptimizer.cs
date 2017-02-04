#define OWNERSHIPTRACKING
using BEPUutilities2.ResourceManagement;
using System;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using static SolverPrototype.SuballocatedBufferPool;

namespace SolverPrototype
{
    /// <summary>
    /// Allocates buffers of data from untyped backing arrays.
    /// </summary>
    public class SuballocatedBufferPool
    {
        byte[][] memoryForPowers;
        Pow2Allocator allocator;
        /// <summary>
        /// Creates a new buffer pool.
        /// </summary>
        /// <param name="initialCapacityPerPowerInBytes">Maximum number of bytes to allocate in each power's pool initially. 
        /// If the specified capacity is fractional at a given power, the capacity is truncated to the next lower block boundary.</param>
        /// <param name="allocator">Backing allocator used to position buffers.</param>
        public SuballocatedBufferPool(int initialCapacityPerPowerInBytes, Pow2Allocator allocator)
        {
            this.allocator = allocator;
            memoryForPowers = new byte[allocator.LargestPower][];
            for (int i = 0; i < memoryForPowers.Length; ++i)
            {
                //Don't really want to deal with both nulls and sizes as a special case, so we just create zero-count arrays if the capacity is to small to fit a whole block.
                var blockCount = initialCapacityPerPowerInBytes >> i;
                memoryForPowers[i] = new byte[blockCount << i];
            }
        }

        public struct BufferRegion
        {
            internal readonly int Power;
            internal readonly int Index;

            /// <summary>
            /// Gets the length of the buffer region.
            /// </summary>
            public int Length
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get { return 1 << Power; }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            internal BufferRegion(int power, int index)
            {
                Power = power;
                Index = index;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Allocate(int power, out BufferRegion region)
        {
            region = new BufferRegion(power, allocator.Allocate(power));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Free(ref BufferRegion region)
        {
            allocator.Free(region.Power, region.Index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref T GetStart<T>(ref BufferRegion region)
        {
            return ref Unsafe.As<byte, T>(ref memoryForPowers[region.Power][region.Index]);
        }
    }

    class ConstraintConnectivityGraph
    {
        //We need a per-body list of constraints because any movement of a body requires updating the connected constraints.
        //Since we have that, we'll also use it to find adjacent bodies. Finding adjacent bodies is required for two purposes:
        //1) Deactivating an island of low energy bodies.
        //2) Optimizing the memory layout of bodies so that connected bodies tend to be adjacent or at least nearby.

        //Conceptually, we could do something simple like a List<List<int>> for a per-body list of constraint handles.
        //An obvious optimization to that is to pool the internal lists to avoid GC overhead, but there is another factor to consider beyond just garbage generation-
        //heap complexity.

        //The goal here is to reduce the cost of a GC. While the engine should never produce garbage itself during regular execution, the application using it might.
        //The cost of a GC is partially related to the number of references it has to track down. An array of reference types requires examining every one of those references.
        //So, if you have 16384 bodies with lists stored in QuickList<int>[] representation, the GC has at least 32768 references it needs to consider!
        //In contrast, if you pack the representation into a single int[] with the lists suballocated from it, the GC doesn't need to scan it- ints aren't reference types.

        //So we could use a custom allocator (like, say, the BEPUutilities Allocator), but there's a much simpler way that is no more wasteful than the QuickList representation 
        //in terms of memory use- preallocated arrays that you can pull fixed size blocks out of. 

        //We'll store references to these memory blocks and treat them like the arrays we'd use for a QuickList. This will be a little ugly due to the typelessness
        //of the backing array, but hopefully in the future we can unify things a bit more.
        struct List
        {
            public BufferRegion Region;
            public int Count;
        }
        List[] constraintLists;

        public ConstraintConnectivityGraph(int initialBodyCountEstimate = 8192, int initialConstraintCountPerBodyEstimate = 8)
        {
            ConstraintCountPerBodyEstimate = initialConstraintCountPerBodyEstimate;
            var capacityEstimate = initialBodyCountEstimate * initialConstraintCountPerBodyEstimate;
            allocator = new Allocator(capacityEstimate,
                new PassthroughBufferPool<ulong>(), new PassthroughBufferPool<Allocator.Allocation>(), new PassthroughBufferPool<int>());
            constraintLists = new
        }

        public void Allocate()
    }
    /// <summary>
    /// Incrementally changes the layout of a set of bodies to minimize the cache misses associated with the solver and other systems that rely on connection following.
    /// </summary>
    class BodyLayoutOptimizer
    {
        Bodies bodies;
        ConstraintConnectivityGraph graph;
        public BodyLayoutOptimizer(Bodies bodies, ConstraintConnectivityGraph graph)
        {
            this.bodies = bodies;
            this.graph = graph;
        }
    }
}
