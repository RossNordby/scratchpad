using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3SOA
    {
        public ScalarWide* X;
        public ScalarWide* Y;
        public ScalarWide* Z;

        public static void Cross(in Vector3SOA a, in Vector3SOA b)
        {
        }

    }
}
