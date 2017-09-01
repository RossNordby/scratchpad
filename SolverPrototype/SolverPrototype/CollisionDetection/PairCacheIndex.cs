using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
    /// <summary>
    /// Packed indirection to data associated with a pair cache entry.
    /// </summary>
    public struct PairCacheIndex
    {
        //TODO: This is a pretty aggressive packing. While most simulations will get by with less than 65536 pairs globally, there are some future simulations 
        //(or modern offline simulations) which could conceivably push beyond 65536 pairs, and at that point you're relying on the distribution of work to avoid an error.
        //If that happens, simply expand this to 8 bytes. It doesn't hurt much to do; we just opted for 4 bytes for the cache's sake.
        uint packed;

        /// <summary>
        /// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
        /// </summary>
        public bool Exists
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (packed & (1 << 31)) > 0; }
        }

        /// <summary>
        /// Gets the worker index that created the entry.
        /// </summary>
        public int Worker
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed >> 21) & 0b11_1111_1111; } //10 bits
        }

        /// <summary>
        /// Gets the type index of the object.
        /// </summary>
        public int Type
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed >> 16) & 0b1_1111; } //5 bits
        }

        /// <summary>
        /// Gets the index of the object.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return (int)(packed & 0x0000FFFF); } //16 bits
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public PairCacheIndex(int worker, int type, int index)
        {
            Debug.Assert(worker >= 0 && worker < (1 << 10), "Do you really have that many threads, or is the index corrupt?");
            Debug.Assert(type >= 0 && type < (1 << 5), "Do you really have that many type indices, or is the index corrupt?");
            Debug.Assert(index >= 0 && index < (1 << 16), "Do you really have that many instances, or is the index corrupt?");
            //Note the inclusion of a set bit in the most significant slot.
            //This encodes that the index was explicitly constructed, so it is a 'real' reference.
            //A default constructed PairCacheIndex will have a 0 in the MSB, so we can use the default constructor for empty references.
            packed = (1u << 31) | (uint)((worker << 21) | (type << 16) | index);
        }

    }
}
