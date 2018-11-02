using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3SOADoofy
    {
        public ScalarSOADoofy X;
        public ScalarSOADoofy Y;
        public ScalarSOADoofy Z;

        //These are pretty doofy SOA usages.
        //Realistically, you would want to execute in bundles to avoid allocating big chunks of temporary memory and evicting local cache over and over.
        //These are just here to contrast with AOSOA.
        public static void Cross(ref Vector3SOADoofy a, ref Vector3SOADoofy b, ArenaPool pool, ref Vector3SOADoofy result)
        {
            var left = new ScalarSOADoofy { Values = pool.Allocate<float>(result.X.Values.Length) };
            var right = new ScalarSOADoofy { Values = pool.Allocate<float>(result.X.Values.Length) };
            ScalarSOADoofy.Multiply(ref a.Y, ref b.Z, ref left);
            ScalarSOADoofy.Multiply(ref a.Z, ref b.Y, ref right);
            ScalarSOADoofy.Subtract(ref left, ref right, ref result.X);

            ScalarSOADoofy.Multiply(ref a.Z, ref b.X, ref left);
            ScalarSOADoofy.Multiply(ref a.X, ref b.Z, ref right);
            ScalarSOADoofy.Subtract(ref left, ref right, ref result.Y);

            ScalarSOADoofy.Multiply(ref a.X, ref b.Y, ref left);
            ScalarSOADoofy.Multiply(ref a.Y, ref b.X, ref right);
            ScalarSOADoofy.Subtract(ref left, ref right, ref result.Z);
        }

        public static void Dot(ref Vector3SOADoofy a, ref Vector3SOADoofy b, ArenaPool pool, ref ScalarSOADoofy result)
        {
            ScalarSOADoofy.Multiply(ref a.X, ref b.X, ref result);
            var temp = new ScalarSOADoofy { Values = pool.Allocate<float>(result.Values.Length) };
            ScalarSOADoofy.Multiply(ref a.Y, ref b.Y, ref temp);
            ScalarSOADoofy.Add(ref result, ref temp, ref result);
            ScalarSOADoofy.Multiply(ref a.Z, ref b.Z, ref temp);
            ScalarSOADoofy.Add(ref result, ref temp, ref result);
        }

        public static void Scale(ref Vector3SOADoofy v, ref ScalarSOADoofy scale, ref Vector3SOADoofy result)
        {
            ScalarSOADoofy.Multiply(ref v.X, ref scale, ref result.X);
            ScalarSOADoofy.Multiply(ref v.Y, ref scale, ref result.Y);
            ScalarSOADoofy.Multiply(ref v.Z, ref scale, ref result.Z);
        }

    }
}
