using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototypeTests.SpecializedTests
{
    static class UnpackCodegenTest
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Test()
        {
            TwoBodyReferences packedReferences;
            packedReferences.IndexA = new Vector<int>(0);
            packedReferences.IndexB = new Vector<int>(0);
            packedReferences.Unpack(0, 4, out var unpacked);
            var start = Stopwatch.GetTimestamp();
            const int iterations = 10000000;
            for (int i = 0; i < iterations; ++i)
            {
                packedReferences.Unpack(0, 4, out var unpacked0);
                packedReferences.Unpack(0, 4, out var unpacked1);
                packedReferences.Unpack(0, 4, out var unpacked2);
                packedReferences.Unpack(0, 4, out var unpacked3);
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Time (ns): {1e9 * (end - start) / ((double)Stopwatch.Frequency * iterations * 4)}");
        }
    }
}
