using BepuUtilities.Memory;

namespace GoingWide
{
    public unsafe class AOSScalar : Benchmark
    {
        struct Input
        {
            public Vector3AOS A;
            public Vector3AOS B;
            public Vector3AOS C;
            public Vector3AOS D;
        }

        Buffer<Input> input;
        Buffer<float> results;

        public AOSScalar()
        {
            input = pool.Allocate<Input>(LaneCount);
            results = pool.Allocate<float>(LaneCount);
        }

        public override void Execute()
        {
            for (int i = 0; i < LaneCount; ++i)
            {
                ref var lane = ref input[i];
                Vector3AOS.CrossScalar(lane.A, lane.B, out var axb);
                Vector3AOS.CrossScalar(lane.C, lane.D, out var cxd);
                var axbDotA = Vector3AOS.DotScalar(axb, lane.A);
                var cxdDotC = Vector3AOS.DotScalar(cxd, lane.C);
                Vector3AOS.ScaleScalar(lane.B, axbDotA, out var left);
                Vector3AOS.ScaleScalar(lane.D, cxdDotC, out var right);
                results[i] = Vector3AOS.DotScalar(left, right);
            }
        }
    }
}
