using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;

namespace GoingWide
{
    public unsafe struct Vector3AOSOALS
    {
        public ScalarWideLS X;
        public ScalarWideLS Y;
        public ScalarWideLS Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(Vector3AOSOALS* a, Vector3AOSOALS* b, Vector3AOSOALS* result)
        {
            ScalarWideLS.Add(&a->X, &b->X, &result->X);
            ScalarWideLS.Add(&a->Y, &b->Y, &result->Y);
            ScalarWideLS.Add(&a->Z, &b->Z, &result->Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(Vector3AOSOALS* a, Vector3AOSOALS* b, Vector3AOSOALS* result)
        {
            ScalarWideLS.Subtract(&a->X, &b->X, &result->X);
            ScalarWideLS.Subtract(&a->Y, &b->Y, &result->Y);
            ScalarWideLS.Subtract(&a->Z, &b->Z, &result->Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(Vector3AOSOALS* a, Vector3AOSOALS* b, Vector3AOSOALS* result)
        {
            Vector3AOSOALS left, right;
            ScalarWideLS.Multiply(&a->Y, &b->Z, &left.X);
            ScalarWideLS.Multiply(&a->Z, &b->X, &left.Y);
            ScalarWideLS.Multiply(&a->X, &b->Y, &left.Z);
            ScalarWideLS.Multiply(&a->Z, &b->Y, &right.X);
            ScalarWideLS.Multiply(&a->X, &b->Z, &right.Y);
            ScalarWideLS.Multiply(&a->Y, &b->X, &right.Z);
            Subtract(&left, &right, result);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Dot(Vector3AOSOALS* a, Vector3AOSOALS* b, ScalarWideLS* result)
        {
            ScalarWideLS x, y, z;
            ScalarWideLS.Multiply(&a->X, &b->X, &x);
            ScalarWideLS.Multiply(&a->Y, &b->Y, &y);
            ScalarWideLS.Multiply(&a->Z, &b->Z, &z);
            ScalarWideLS.Add(&x, &y, result);
            ScalarWideLS.Add(result, &z, result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(Vector3AOSOALS* v, ScalarWideLS* scale, Vector3AOSOALS* result)
        {
            ScalarWideLS.Multiply(&v->X, scale, &result->X);
            ScalarWideLS.Multiply(&v->Y, scale, &result->Y);
            ScalarWideLS.Multiply(&v->Z, scale, &result->Z);
        }

    }
}
