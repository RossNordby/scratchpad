using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace GoingWide
{
    public unsafe class SOA : Benchmark
    {
        Vector3SOA a, b, c, d;
        //For simplicity, intermediates are preallocated.
        Vector3SOA v0, v1;
        float* dot;
        float* result;

        Vector3SOA CreateVector()
        {
            Vector3SOA v;
            v.X = (ScalarWideU*)pool.Allocate(4 * LaneCount);
            v.Y = (ScalarWideU*)pool.Allocate(4 * LaneCount);
            v.Z = (ScalarWideU*)pool.Allocate(4 * LaneCount);
            return v;
        }
        public SOA()
        {
            a = CreateVector();
            b = CreateVector();
            c = CreateVector();
            d = CreateVector();

            v0 = CreateVector();
            v1 = CreateVector();
            dot = (float*)pool.Allocate(4 * LaneCount);
            result = (float*)pool.Allocate(4 * LaneCount);
        }

        public override void Execute()
        {
            //We assume the lane count is evenly divisible here. No "remainder" post-loop.
            const int bundleCount = LaneCount / 8;
            for (int i = 0; i < bundleCount; ++i)
            {
                ref var ax = ref a.X[i];
                ref var ay = ref a.Y[i];
                ref var az = ref a.Z[i];
                ref var bx = ref b.X[i];
                ref var by = ref b.Y[i];
                ref var bz = ref b.Z[i];

                //    Vector3Shared.Cross(ref ax, ref ay, ref az, ref bx, ref by, ref bz, out r0X, ref *v0.Y, ref *v0.Z);
                //    Vector3Shared.Dot(ref *a.X, ref *a.Y, ref *a.Z, ref *b.X, ref *b.Y, ref *b.Z, )
                //Vector3Shared.Cross(
                //    ref *(ScalarWide*)a.X, ref *(ScalarWide*)a.Y, ref *(ScalarWide*)a.Z,
                //    ref *(ScalarWide*)b.X, ref *(ScalarWide*)b.Y, ref *(ScalarWide*)b.Z,
                //    ref *(ScalarWide*)v0.X, ref *(ScalarWide*)v0.Y, ref *(ScalarWide*)v0.Z);
            }
        }
    }
}
