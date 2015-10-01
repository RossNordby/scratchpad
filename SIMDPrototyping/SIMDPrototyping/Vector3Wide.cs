using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public struct Vector3Wide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe Vector3Wide(float[] x, float[] y, float[] z)
        {
            X = new Vector<float>(x);
            Y = new Vector<float>(y);
            Z = new Vector<float>(z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe Vector3Wide(float x, float y, float z)
        {
            X = new Vector<float>(x);
            Y = new Vector<float>(y);
            Z = new Vector<float>(z);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe Vector3Wide(ref Vector3 v)
        {
            X = new Vector<float>(v.X);
            Y = new Vector<float>(v.Y);
            Z = new Vector<float>(v.Z);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X - b.X;
            result.Y = a.Y - b.Y;
            result.Z = a.Z - b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref Vector3Wide a, ref Vector<float> b, out Vector3Wide result)
        {
            result.X = a.X * b;
            result.Y = a.Y * b;
            result.Z = a.Z * b;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref Vector3Wide a, float b, out Vector3Wide result)
        {
            result.X = a.X * b;
            result.Y = a.Y * b;
            result.Z = a.Z * b;
        }




        /// <summary>
        /// Performs a 4-wide cross product that assumes the result is a separate location.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void ConditionalSelect(ref Vector<int> mask, ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = Vector.ConditionalSelect(mask, a.X, b.X);
            result.Y = Vector.ConditionalSelect(mask, a.Y, b.Y);
            result.Z = Vector.ConditionalSelect(mask, a.Z, b.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void Max(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = Vector.Max(a.X, b.X);
            result.Y = Vector.Max(a.Y, b.Y);
            result.Z = Vector.Max(a.Z, b.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void Min(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = Vector.Min(a.X, b.X);
            result.Y = Vector.Min(a.Y, b.Y);
            result.Z = Vector.Min(a.Z, b.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref Vector3Wide v, out Vector3Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static unsafe void Transpose(ref Vector3Wide source, Vector3* result)
        {
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                result[i] = new Vector3(source.X[i], source.Y[i], source.Z[i]);
            }
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void Dot(ref Vector3Wide a, ref Vector3Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }
    }
}
