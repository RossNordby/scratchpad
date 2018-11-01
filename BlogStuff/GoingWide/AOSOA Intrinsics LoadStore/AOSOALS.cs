using BepuUtilities.Memory;
using System;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe class AOSOALS : Benchmark
    {
        struct Input
        {
            public Vector3AOSOALS A;
            public Vector3AOSOALS B;
            public Vector3AOSOALS C;
            public Vector3AOSOALS D;
        }

        Buffer<Input> input;
        Buffer<ScalarWideLS> results;

        public AOSOALS()
        {
            input = pool.Allocate<Input>(LaneCount / ScalarWideLS.BundleSize);
            results = pool.Allocate<ScalarWideLS>(LaneCount / ScalarWideLS.BundleSize);
        }
        public override void Execute()
        {
            for (int i = 0; i < LaneCount / ScalarWideLS.BundleSize; ++i)
            {
                var lane = (Input*)input.Memory + i;
                Vector3AOSOALS axb, cxd;
                Vector3AOSOALS.Cross(&lane->A, &lane->B, &axb);
                Vector3AOSOALS.Cross(&lane->C, &lane->D, &cxd);
                ScalarWideLS axbDotA, cxdDotC;
                Vector3AOSOALS.Dot(&axb, &lane->A, &axbDotA);
                Vector3AOSOALS.Dot(&cxd, &lane->C, &cxdDotC);
                Vector3AOSOALS left, right;
                Vector3AOSOALS.Scale(&lane->B, &axbDotA, &left);
                Vector3AOSOALS.Scale(&lane->D, &cxdDotC, &right);
                Vector3AOSOALS.Dot(&left, &right, (ScalarWideLS*)results.Memory + i);
            }
        }
    }
}
