using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities
{
    /// <summary>
    /// Extra functionality for the Vector3 struct.
    /// </summary>
    /// <remarks>Hopefully, all of this should eventually go away as the System.Numerics.Vectors improves.</remarks>
    public struct Vector3x
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(ref Vector3 a, ref Vector3 b, out Vector3 result)
        {
            result = new Vector3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        }

        struct V
        {
            public float X, Y, Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Cross(Vector3* a, Vector3* b, out Vector3 result)
        {
            V* af = (V*)a;
            V* bf = (V*)b;
            result = new Vector3(
                af->Y * bf->Z - af->Z * bf->Y,
                af->Z * bf->X - af->X * bf->Z,
                af->X * bf->Y - af->Y * bf->X);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public unsafe static void Cross(Vector3* a, Vector3* b, Vector3* result)
        //{
        //    V* af = (V*)a;
        //    V* bf = (V*)b;
        //    V* rf = (V*)result;
        //    rf->X = af->Y * bf->Z - af->Z * bf->Y;
        //    rf->Y = af->Z * bf->X - af->X * bf->Z;
        //    rf->Z = af->X * bf->Y - af->Y * bf->X;
        //}

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void Cross(V* a, V* b, V* result)
        {
            result->X = a->Y * b->Z - a->Z * b->Y;
            result->Y = a->Z * b->X - a->X * b->Z;
            result->Z = a->X * b->Y - a->Y * b->X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Cross(Vector3* a, Vector3* b, Vector3* result)
        {
            Cross((V*)a, (V*)b, (V*)result);
        }
    }
}
