using BepuUtilities.Memory;
using System;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe class AOSOAU : Benchmark
    {
        struct Input
        {
            public Vector3AOSOAU A;
            public Vector3AOSOAU B;
            public Vector3AOSOAU C;
            public Vector3AOSOAU D;
        }

        Buffer<Input> input;
        Buffer<ScalarWideU> results;

        public AOSOAU()
        {
            input = pool.Allocate<Input>(LaneCount / ScalarWideU.BundleSize);
            results = pool.Allocate<ScalarWideU>(LaneCount / ScalarWideU.BundleSize);
        }
        public override void Execute()
        {
            for (int i = 0; i < LaneCount / ScalarWideU.BundleSize; ++i)
            {
                var lane = (Input*)input.Memory + i;
                Vector3AOSOAU axb, cxd;
                Vector3AOSOAU.Cross(&lane->A, &lane->B, &axb);
                Vector3AOSOAU.Cross(&lane->C, &lane->D, &cxd);
                ScalarWideU axbDotA, cxdDotC;
                Vector3AOSOAU.Dot(&axb, &lane->A, &axbDotA);
                Vector3AOSOAU.Dot(&cxd, &lane->C, &cxdDotC);
                Vector3AOSOAU left, right;
                Vector3AOSOAU.Scale(&lane->B, &axbDotA, &left);
                Vector3AOSOAU.Scale(&lane->D, &cxdDotC, &right);
                Vector3AOSOAU.Dot(&left, &right, (ScalarWideU*)results.Memory + i);
            }
        }
    }
}
