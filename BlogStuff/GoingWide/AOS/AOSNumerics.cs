using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe class AOSNumerics : Benchmark
    {
        struct Input
        {
            public Vector3 A;
            public Vector3 B;
            public Vector3 C;
            public Vector3 D;
        }

        Buffer<Input> input;
        Buffer<float> results;

        public AOSNumerics()
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
                    var axb = Vector3.Cross(lane.A, lane.B);
                    var cxd = Vector3.Cross(lane.C, lane.D);
                    var axbDotA = Vector3.Dot(axb, lane.A);
                    var cxdDotC = Vector3.Dot(cxd, lane.C);
                    var left = lane.B * axbDotA;
                    var right = lane.D * cxdDotC;
                    results[i] = Vector3.Dot(left, right);
                }
            }
        }
    }
}
