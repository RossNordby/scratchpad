using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities
{
    /// <summary>
    /// Provides SIMD-aware 4x4 matrix math.
    /// </summary>
    public struct MatrixSIMD
    {
        /// <summary>
        /// Row 1 of the matrix.
        /// </summary>
        public Vector4 X;
        /// <summary>
        /// Row 2 of the matrix.
        /// </summary>
        public Vector4 Y;
        /// <summary>
        /// Row 3 of the matrix.
        /// </summary>
        public Vector4 Z;
        /// <summary>
        /// Row 4 of the matrix.
        /// </summary>
        public Vector4 W;

        public static MatrixSIMD Identity
        {
            get
            {
                MatrixSIMD result;
                result.X = new Vector4(1, 0, 0, 0);
                result.Y = new Vector4(0, 1, 0, 0);
                result.Z = new Vector4(0, 0, 1, 0);
                result.W = new Vector4(0, 0, 0, 1);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformTranspose(ref Vector4 v, ref MatrixSIMD m, out Vector4 result)
        {
            //Assume row vector.
            //It multiplies across the columns.
            result = new Vector4(
                Vector4.Dot(v, m.X),
                Vector4.Dot(v, m.Y),
                Vector4.Dot(v, m.Z),
                Vector4.Dot(v, m.W));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector4 v, ref MatrixSIMD m, out Vector4 result)
        {
            //Assume column vector.
            //It multiplies across the rows.
            var x = new Vector4(v.X);
            var y = new Vector4(v.Y);
            var z = new Vector4(v.Z);
            var w = new Vector4(v.W);
            result = m.X * x + m.Y * y + m.Z * z + m.W * w;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref MatrixSIMD a, ref MatrixSIMD b, out MatrixSIMD result)
        {
            {
                var x = new Vector4(a.X.X);
                var y = new Vector4(a.X.Y);
                var z = new Vector4(a.X.Z);
                var w = new Vector4(a.X.W);
                result.X = x * b.X + y * b.Y + z * b.Z + w * b.W;
            }

            {
                var x = new Vector4(a.Y.X);
                var y = new Vector4(a.Y.Y);
                var z = new Vector4(a.Y.Z);
                var w = new Vector4(a.Y.W);
                result.Y = x * b.X + y * b.Y + z * b.Z + w * b.W;
            }

            {
                var x = new Vector4(a.Z.X);
                var y = new Vector4(a.Z.Y);
                var z = new Vector4(a.Z.Z);
                var w = new Vector4(a.Z.W);
                result.Z = x * b.X + y * b.Y + z * b.Z + w * b.W;
            }

            {
                var x = new Vector4(a.W.X);
                var y = new Vector4(a.W.Y);
                var z = new Vector4(a.W.Z);
                var w = new Vector4(a.W.W);
                result.W = x * b.X + y * b.Y + z * b.Z + w * b.W;
            }
        }

    }
}
