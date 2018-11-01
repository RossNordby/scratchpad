using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3SOA
    {
        public ScalarWideU* X;
        public ScalarWideU* Y;
        public ScalarWideU* Z;

        public static void Cross(in Vector3SOA a, in Vector3SOA b)
        {
        }

    }
}
