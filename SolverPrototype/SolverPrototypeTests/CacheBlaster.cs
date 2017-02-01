using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests
{
    public static class CacheBlaster
    {
        const int longCount = (1 << 21); //16.7MB is bigger than most desktop last level caches. You'll want to pick something higher if you're running this on some ginormo xeon.
        const int mask = longCount - 1;
        static long[] readblob = new long[longCount];
        static long[] writeblob = new long[longCount];

        /// <summary>
        /// Attempts to evict most or all of the cache levels to simulate a cold start.
        /// Doesn't do a whole lot for simulations so large that they significantly exceed the cache size.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoOptimization)]
        public static void Blast()
        {
            //We don't have a guarantee that the processor is using pure LRU replacement. Some modern processors are a little trickier.
            //Scrambling the accesses should make it harder for the CPU to keep stuff cached.
            for (int i = 0; i < longCount; ++i)
            {
                writeblob[i] = readblob[(i * 104395303) & mask];
            }
        }
    }
}
