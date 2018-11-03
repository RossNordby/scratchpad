using BepuUtilities.Memory;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe class AOSSSE : Benchmark
    {
        struct Input
        {
            public Vector128<float> A;
            public Vector128<float> B;
            public Vector128<float> C;
            public Vector128<float> D;
        }

        Buffer<Input> input;
        Buffer<float> results;

        public AOSSSE()
        {
            input = pool.Allocate<Input>(LaneCount);
            results = pool.Allocate<float>(LaneCount);
        }

        public override void Execute()
        {
            if (Sse41.IsSupported)
            {
                for (int i = 0; i < LaneCount; ++i)
                {
                    ref var lane = ref input[i];
                    Vector3AOS.Cross3ShuffleSSE(lane.A, lane.B, out var axb);
                    Vector3AOS.Cross3ShuffleSSE(lane.C, lane.D, out var cxd);
                    Vector3AOS.DotSSE(axb, lane.A, out var axbDotA);
                    Vector3AOS.DotSSE(cxd, lane.C, out var cxdDotC);
                    Vector3AOS.ScaleSSE(lane.B, axbDotA, out var left);
                    Vector3AOS.ScaleSSE(lane.D, cxdDotC, out var right);
                    Vector3AOS.DotSSE(left, right, out var result);
                    results[i] = Sse41.Extract(result, 0);
                }
            }
        }
    }
}
