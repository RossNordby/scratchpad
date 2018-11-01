using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;

namespace GoingWide
{
    public unsafe class SOA : Benchmark
    {
        Vector3SOA a, b, c, d;
        ScalarSOA result;

        Vector3SOA CreateVector()
        {
            Vector3SOA v;
            v.X = new ScalarSOA { Values = pool.Allocate<float>(LaneCount) };
            v.Y = new ScalarSOA { Values = pool.Allocate<float>(LaneCount) };
            v.Z = new ScalarSOA { Values = pool.Allocate<float>(LaneCount) };
            return v;
        }
        public SOA()
        {
            a = CreateVector();
            b = CreateVector();
            c = CreateVector();
            d = CreateVector();

            result = new ScalarSOA { Values = pool.Allocate<float>(LaneCount) };
        }

        public override void Execute()
        {
            var left = CreateVector();
            var dot = new ScalarSOA { Values = pool.Allocate<float>(LaneCount) };
            Vector3SOA.Cross(ref a, ref b, pool, ref left);
            Vector3SOA.Dot(ref left, ref a, pool, ref dot);
            Vector3SOA.Scale(ref b, ref dot, ref left);

            var right = CreateVector();
            Vector3SOA.Cross(ref c, ref d, pool, ref right);
            Vector3SOA.Dot(ref right, ref c, pool, ref dot);
            Vector3SOA.Scale(ref d, ref dot, ref right);

            Vector3SOA.Dot(ref left, ref right, pool, ref result);
        }
    }
}
