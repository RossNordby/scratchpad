using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3AOSOANumerics
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(in Vector3AOSOANumerics a, in Vector3AOSOANumerics b, out Vector3AOSOANumerics result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(in Vector3AOSOANumerics a, in Vector3AOSOANumerics b, out Vector3AOSOANumerics result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(in Vector3AOSOANumerics a, in Vector3AOSOANumerics b, out Vector3AOSOANumerics result)
        {
            //This will fail if the result reference is actually a or b! 
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(in Vector3AOSOANumerics a, in Vector3AOSOANumerics b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(in Vector3AOSOANumerics v, Vector<float> scale, out Vector3AOSOANumerics result)
        {
            result.X = v.X * scale;
            result.Y = v.Y * scale;
            result.Z = v.Z * scale;
        }

    }
}
