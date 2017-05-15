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
        public static void CreateFrom(ref Vector3 source, out Vector3Wide broadcasted)
        {
            broadcasted.X = new Vector<float>(source.X);
            broadcasted.Y = new Vector<float>(source.Y);
            broadcasted.Z = new Vector<float>(source.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLengthSquared(ref Vector3Wide v, out Vector<float> lengthSquared)
        {
            lengthSquared = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLength(ref Vector3Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
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
        public static void Dot(ref Vector3Wide a, ref Vector3Wide b, out Vector<float> result)
        {
            result = a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Vector3Wide vector, ref Vector<float> scalar, out Vector3Wide result)
        {
            result.X = vector.X * scalar;
            result.Y = vector.Y * scalar;
            result.Z = vector.Z * scalar;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Abs(ref Vector3Wide vector, out Vector3Wide result)
        {
            result.X = Vector.Abs(vector.X);
            result.Y = Vector.Abs(vector.Y);
            result.Z = Vector.Abs(vector.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref Vector3Wide v, out Vector3Wide result)
        {
            result.X = -v.X;
            result.Y = -v.Y;
            result.Z = -v.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide Negate(ref Vector3Wide v)
        {
            v.X = -v.X;
            v.Y = -v.Y;
            v.Z = -v.Z;
            return ref v;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Length(ref Vector3Wide v, out Vector<float> length)
        {
            length = Vector.SquareRoot(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Distance(ref Vector3Wide a, ref Vector3Wide b, out Vector<float> distance)
        {
            Subtract(ref b, ref a, out var offset);
            Length(ref offset, out distance);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Normalize(ref Vector3Wide v, out Vector3Wide result)
        {
            GetLength(ref v, out var length);
            var scale = Vector<float>.One / length;
            Scale(ref v, ref scale, out result);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ConditionalSelect(ref Vector<int> condition, ref Vector3Wide left, ref Vector3Wide right, out Vector3Wide result)
        {
            result.X = Vector.ConditionalSelect(condition, left.X, right.X);
            result.Y = Vector.ConditionalSelect(condition, left.Y, right.Y);
            result.Z = Vector.ConditionalSelect(condition, left.Z, right.Z);
        }
    }
}
