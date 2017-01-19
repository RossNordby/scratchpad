using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct Vector3Wide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;

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
        public static void Dot(ref Vector3Wide a, ref Vector3Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref Vector3Wide vector, ref Vector<float> scalar, out Vector3Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref Vector3Wide v, out Vector3Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CrossWithoutOverlap(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            //This will fail if the result reference is actually a or b! 
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            CrossWithoutOverlap(ref a, ref b, out var temp);
            result = temp;
        }

    }
}
