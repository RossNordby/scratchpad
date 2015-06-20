using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public struct Vector3Width4
    {
        public Vector4 X;
        public Vector4 Y;
        public Vector4 Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3Width4(ref Vector3 v1, ref Vector3 v2, ref Vector3 v3, ref Vector3 v4)
        {
            X = new Vector4(v1.X, v2.X, v3.X, v4.X);
            Y = new Vector4(v1.Y, v2.Y, v3.Y, v4.Y);
            Z = new Vector4(v1.Z, v2.Z, v3.Z, v4.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Vector3Width4 a, ref Vector3Width4 b, out Vector3Width4 result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Vector3Width4 a, ref Vector3Width4 b, out Vector3Width4 result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref Vector3Width4 a, ref Vector4 b, out Vector3Width4 result)
        {
            result.X = a.X * b;
            result.Y = a.Y * b;
            result.Z = a.Z * b;
        }


        /// <summary>
        /// Performs a 4-wide cross product that assumes the result is a separate location.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(ref Vector3Width4 a, ref Vector3Width4 b, out Vector3Width4 result)
        {
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref Vector3Width4 v, out Vector3Width4 result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void Transpose(ref Vector3Width4 linearChangeA, Vector3* linearChangesA)
        {
            linearChangesA[0] = new Vector3(linearChangeA.X.X, linearChangeA.Y.X, linearChangeA.Z.X);
            linearChangesA[1] = new Vector3(linearChangeA.X.Y, linearChangeA.Y.Y, linearChangeA.Z.Y);
            linearChangesA[2] = new Vector3(linearChangeA.X.Z, linearChangeA.Y.Z, linearChangeA.Z.Z);
            linearChangesA[3] = new Vector3(linearChangeA.X.W, linearChangeA.Y.W, linearChangeA.Z.W);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void Transpose(ref Vector3Width4 linearChangeA, out Vector3 a, out Vector3 b, out Vector3 c, out Vector3 d)
        {
            a = new Vector3(linearChangeA.X.X, linearChangeA.Y.X, linearChangeA.Z.X);
            b = new Vector3(linearChangeA.X.Y, linearChangeA.Y.Y, linearChangeA.Z.Y);
            c = new Vector3(linearChangeA.X.Z, linearChangeA.Y.Z, linearChangeA.Z.Z);
            d = new Vector3(linearChangeA.X.W, linearChangeA.Y.W, linearChangeA.Z.W);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void Dot(ref Vector3Width4 a, ref Vector3Width4 b, out Vector4 result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }
    }
}
