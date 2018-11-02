using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;

namespace GoingWide
{
    public unsafe class SOADoofy : Benchmark
    {
        Vector3SOADoofy a, b, c, d;
        ScalarSOADoofy result;

        Vector3SOADoofy CreateVector()
        {
            Vector3SOADoofy v;
            v.X = new ScalarSOADoofy { Values = pool.Allocate<float>(LaneCount) };
            v.Y = new ScalarSOADoofy { Values = pool.Allocate<float>(LaneCount) };
            v.Z = new ScalarSOADoofy { Values = pool.Allocate<float>(LaneCount) };
            return v;
        }
        public SOADoofy()
        {
            a = CreateVector();
            b = CreateVector();
            c = CreateVector();
            d = CreateVector();

            result = new ScalarSOADoofy { Values = pool.Allocate<float>(LaneCount) };
        }

        public override void Execute()
        {
            var left = CreateVector();
            var dot = new ScalarSOADoofy { Values = pool.Allocate<float>(LaneCount) };
            Vector3SOADoofy.Cross(ref a, ref b, pool, ref left);
            Vector3SOADoofy.Dot(ref left, ref a, pool, ref dot);
            Vector3SOADoofy.Scale(ref b, ref dot, ref left);

            var right = CreateVector();
            Vector3SOADoofy.Cross(ref c, ref d, pool, ref right);
            Vector3SOADoofy.Dot(ref right, ref c, pool, ref dot);
            Vector3SOADoofy.Scale(ref d, ref dot, ref right);

            Vector3SOADoofy.Dot(ref left, ref right, pool, ref result);
        }
    }
}
