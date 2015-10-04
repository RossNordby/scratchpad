using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities
{
    /// <summary>
    /// Provides SIMD-aware 4x4 matrix math.
    /// </summary>
    /// <remarks>
    /// All functions assume row vectors.
    /// </remarks>
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
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

        struct M
        {
            public float M11, M12, M13, M14;
            public float M21, M22, M23, M24;
            public float M31, M32, M33, M34;
            public float M41, M42, M43, M44;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void Transpose(M* m, M* transposed)
        {
            //A weird function! Why?
            //1) Missing some helpful instructions for actual SIMD accelerated transposition.
            //2) Difficult to get SIMD types to generate competitive codegen due to lots of componentwise access.

            float m12 = m->M12;
            float m13 = m->M13;
            float m14 = m->M14;
            float m23 = m->M23;
            float m24 = m->M24;
            float m34 = m->M34;
            transposed->M11 = m->M11;
            transposed->M12 = m->M21;
            transposed->M13 = m->M31;
            transposed->M14 = m->M41;

            transposed->M21 = m12;
            transposed->M22 = m->M22;
            transposed->M23 = m->M32;
            transposed->M24 = m->M42;

            transposed->M31 = m13;
            transposed->M32 = m23;
            transposed->M33 = m->M33;
            transposed->M34 = m->M43;

            transposed->M41 = m14;
            transposed->M42 = m24;
            transposed->M43 = m34;
            transposed->M44 = m->M44;
            
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Transpose(MatrixSIMD* m, MatrixSIMD* transposed)
        {
            Transpose((M*)m, (M*)transposed);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transpose(ref MatrixSIMD m, out MatrixSIMD transposed)
        {
            //Not an ideal implementation. Shuffles would be handy.
            
            var xy = m.X.Y;
            var xz = m.X.Z;
            var xw = m.X.W;
            var yz = m.Y.Z;
            var yw = m.Y.W;
            var zw = m.Z.W;
            transposed.X = new Vector4(m.X.X, m.Y.X, m.Z.X, m.W.X);
            transposed.Y = new Vector4(xy, m.Y.Y, m.Z.Y, m.W.Y);
            transposed.Z = new Vector4(xz, yz, m.Z.Z, m.W.Z);
            transposed.W = new Vector4(xw, yw, zw, m.W.W);
        }

        /// <summary>
        /// Transforms a vector with a transposed matrix.
        /// </summary>
        /// <param name="v">Row vector to transform.</param>
        /// <param name="m">Matrix whose transpose will be applied to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformTranspose(ref Vector4 v, ref MatrixSIMD m, out Vector4 result)
        {
            result = new Vector4(
                Vector4.Dot(v, m.X),
                Vector4.Dot(v, m.Y),
                Vector4.Dot(v, m.Z),
                Vector4.Dot(v, m.W));
        }

        /// <summary>
        /// Transforms a vector with a matrix.
        /// </summary>
        /// <param name="v">Row vector to transform.</param>
        /// <param name="m">Matrix to apply to the vector.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector4 v, ref MatrixSIMD m, out Vector4 result)
        {
            var x = new Vector4(v.X);
            var y = new Vector4(v.Y);
            var z = new Vector4(v.Z);
            var w = new Vector4(v.W);
            result = m.X * x + m.Y * y + m.Z * z + m.W * w;
        }



        /// <summary>
        /// Multiplies a matrix by another matrix.
        /// </summary>
        /// <param name="a">First matrix.</param>
        /// <param name="b">Second matrix.</param>
        /// <param name="result">Result of the matrix multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref MatrixSIMD a, ref MatrixSIMD b, out MatrixSIMD result)
        {
            var bX = b.X;
            var bY = b.Y;
            var bZ = b.Z;
            {
                var x = new Vector4(a.X.X);
                var y = new Vector4(a.X.Y);
                var z = new Vector4(a.X.Z);
                var w = new Vector4(a.X.W);
                result.X = (x * bX + y * bY) + (z * bZ + w * b.W);
            }

            {
                var x = new Vector4(a.Y.X);
                var y = new Vector4(a.Y.Y);
                var z = new Vector4(a.Y.Z);
                var w = new Vector4(a.Y.W);
                result.Y = (x * bX + y * bY) + (z * bZ + w * b.W);
            }

            {
                var x = new Vector4(a.Z.X);
                var y = new Vector4(a.Z.Y);
                var z = new Vector4(a.Z.Z);
                var w = new Vector4(a.Z.W);
                result.Z = (x * bX + y * bY) + (z * bZ + w * b.W);
            }

            {
                var x = new Vector4(a.W.X);
                var y = new Vector4(a.W.Y);
                var z = new Vector4(a.W.Z);
                var w = new Vector4(a.W.W);
                result.W = (x * bX + y * bY) + (z * bZ + w * b.W);
            }
        }

    }
}
