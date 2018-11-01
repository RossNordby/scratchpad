using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3SOA
    {
        public ScalarSOA X;
        public ScalarSOA Y;
        public ScalarSOA Z;

        //These are pretty doofy SOA usages.
        //Realistically, you would want to execute in bundles to avoid allocating big chunks of temporary memory and evicting local cache over and over.
        //These are just here to contrast with AOSOA.
        public static void Cross(ref Vector3SOA a, ref Vector3SOA b, ArenaPool pool, ref Vector3SOA result)
        {
            var left = new ScalarSOA { Values = pool.Allocate<float>(result.X.Values.Length) };
            var right = new ScalarSOA { Values = pool.Allocate<float>(result.X.Values.Length) };
            ScalarSOA.Multiply(ref a.Y, ref b.Z, ref left);
            ScalarSOA.Multiply(ref a.Z, ref b.Y, ref right);
            ScalarSOA.Subtract(ref left, ref right, ref result.X);

            ScalarSOA.Multiply(ref a.Z, ref b.X, ref left);
            ScalarSOA.Multiply(ref a.X, ref b.Z, ref right);
            ScalarSOA.Subtract(ref left, ref right, ref result.Y);

            ScalarSOA.Multiply(ref a.X, ref b.Y, ref left);
            ScalarSOA.Multiply(ref a.Y, ref b.X, ref right);
            ScalarSOA.Subtract(ref left, ref right, ref result.Z);
        }

        public static void Dot(ref Vector3SOA a, ref Vector3SOA b, ArenaPool pool, ref ScalarSOA result)
        {
            ScalarSOA.Multiply(ref a.X, ref b.X, ref result);
            var temp = new ScalarSOA { Values = pool.Allocate<float>(result.Values.Length) };
            ScalarSOA.Multiply(ref a.Y, ref b.Y, ref temp);
            ScalarSOA.Add(ref result, ref temp, ref result);
            ScalarSOA.Multiply(ref a.Z, ref b.Z, ref temp);
            ScalarSOA.Add(ref result, ref temp, ref result);
        }

        public static void Scale(ref Vector3SOA v, ref ScalarSOA scale, ref Vector3SOA result)
        {
            ScalarSOA.Multiply(ref v.X, ref scale, ref result.X);
            ScalarSOA.Multiply(ref v.Y, ref scale, ref result.Y);
            ScalarSOA.Multiply(ref v.Z, ref scale, ref result.Z);
        }

    }
}
