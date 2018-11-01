using BepuUtilities.Memory;
using System;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe class AOSOA : Benchmark
    {
        struct Input
        {
            public Vector3AOSOA A;
            public Vector3AOSOA B;
            public Vector3AOSOA C;
            public Vector3AOSOA D;
        }

        Buffer<Input> input;
        Buffer<ScalarWide> results;

        public AOSOA()
        {
            input = pool.Allocate<Input>(LaneCount / ScalarWide.BundleSize);
            results = pool.Allocate<ScalarWide>(LaneCount / ScalarWide.BundleSize);
        }
        public override void Execute()
        {
            for (int i = 0; i < LaneCount / ScalarWide.BundleSize; ++i)
            {
                var lane = (Input*)input.Memory + i;
                Vector3AOSOA axb, cxd;
                Vector3AOSOA.Cross(&lane->A, &lane->B, &axb);
                Vector3AOSOA.Cross(&lane->C, &lane->D, &cxd);
                ScalarWide axbDotA, cxdDotC;
                Vector3AOSOA.Dot(&axb, &lane->A, &axbDotA);
                Vector3AOSOA.Dot(&cxd, &lane->C, &cxdDotC);
                Vector3AOSOA left, right;
                Vector3AOSOA.Scale(&lane->B, &axbDotA, &left);
                Vector3AOSOA.Scale(&lane->D, &cxdDotC, &right);
                Vector3AOSOA.Dot(&left, &right, (ScalarWide*)results.Memory + i);
            }
        }
    }
}
