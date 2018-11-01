using BepuUtilities.Memory;
using System.Numerics;

namespace GoingWide
{
    public unsafe class AOSOANumerics : Benchmark
    {
        struct Input
        {
            public Vector3AOSOANumerics A;
            public Vector3AOSOANumerics B;
            public Vector3AOSOANumerics C;
            public Vector3AOSOANumerics D;
        }

        Buffer<Input> input;
        Buffer<Vector<float>> results;

        public AOSOANumerics()
        {
            input = pool.Allocate<Input>(LaneCount / Vector<float>.Count);
            results = pool.Allocate<Vector<float>>(LaneCount / Vector<float>.Count);
        }
        public override void Execute()
        {
            for (int i = 0; i < LaneCount / Vector<float>.Count; ++i)
            {
                ref var lane = ref input[i];
                Vector3AOSOANumerics.Cross(lane.A, lane.B, out var axb);
                Vector3AOSOANumerics.Cross(lane.C, lane.D, out var cxd);
                Vector3AOSOANumerics.Dot(axb, lane.A, out var axbDotA);
                Vector3AOSOANumerics.Dot(cxd, lane.C, out var cxdDotC);
                Vector3AOSOANumerics.Scale(lane.B, axbDotA, out var left);
                Vector3AOSOANumerics.Scale(lane.D, cxdDotC, out var right);
                Vector3AOSOANumerics.Dot(left, right, out results[i]);
            }
        }
    }
}
