using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities
{
    /// <summary>
    /// 3 row, 3 column matrix.
    /// </summary>
    public struct Matrix3x3SIMD
    {
        public Vector3 X;
        public Vector3 Y;
        public Vector3 Z;


        /// <summary>
        /// Gets the 3x3 identity matrix.
        /// </summary>
        public static Matrix3x3SIMD Identity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Matrix3x3SIMD toReturn;
                toReturn.X = new Vector3(1, 0, 0);
                toReturn.Y = new Vector3(0, 1, 0);
                toReturn.Z = new Vector3(0, 0, 1);
                return toReturn;
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void Transpose(M* m, M* transposed)
        {
            //A weird function! Why?
            //1) Missing some helpful instructions for actual SIMD accelerated transposition.
            //2) Difficult to get SIMD types to generate competitive codegen due to lots of componentwise access.

            float m12 = m->M12;
            float m13 = m->M13;
            float m23 = m->M23;
            transposed->M11 = m->M11;
            transposed->M12 = m->M21;
            transposed->M13 = m->M31;

            transposed->M21 = m12;
            transposed->M22 = m->M22;
            transposed->M23 = m->M32;

            transposed->M31 = m13;
            transposed->M32 = m23;
            transposed->M33 = m->M33;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Transpose(Matrix3x3SIMD* m, Matrix3x3SIMD* transposed)
        {
            Transpose((M*)m, (M*)transposed);
        }

        /// <summary>                                                                                                
        /// Computes the transposed matrix of a matrix.                                                              
        /// </summary>                                                                                               
        /// <param name="m">Matrix to transpose.</param>                                                             
        /// <param name="transposed">Transposed matrix.</param>                                                      
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transpose(ref Matrix3x3SIMD m, out Matrix3x3SIMD transposed)
        {
            var xy = m.X.Y;
            var xz = m.X.Z;
            var yz = m.Y.Z;
            transposed.X = new Vector3(m.X.X, m.Y.X, m.Z.X);
            transposed.Y = new Vector3(xy, m.Y.Y, m.Z.Y);
            transposed.Z = new Vector3(xz, yz, m.Z.Z);
        }


        /// <summary>
        /// Transforms the vector by the matrix.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="m">Matrix to use as the transformation.</param>
        /// <param name="result">Product of the transformation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3 v, ref Matrix3x3SIMD m, out Vector3 result)
        {
            var x = new Vector3(v.X);
            var y = new Vector3(v.Y);
            var z = new Vector3(v.Z);
            result = m.X * x + m.Y * y + m.Z * z;
        }

        /// <summary>
        /// Transforms the vector by the matrix's transpose.
        /// </summary>
        /// <param name="v">Vector3 to transform.</param>
        /// <param name="m">Matrix to use as the transformation transpose.</param>
        /// <param name="result">Product of the transformation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformTranspose(ref Vector3 v, ref Matrix3x3SIMD m, out Vector3 result)
        {
            result = new Vector3(
                Vector3.Dot(v, m.X),
                Vector3.Dot(v, m.Y),
                Vector3.Dot(v, m.Z));
        }

        struct M
        {
            public float M11, M12, M13;
            public float M21, M22, M23;
            public float M31, M32, M33;
        }

        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref Matrix3x3SIMD a, ref Matrix3x3SIMD b, out Matrix3x3SIMD result)
        {
            var bX = b.X;
            var bY = b.Y;
            {
                var x = new Vector3(a.X.X);
                var y = new Vector3(a.X.Y);
                var z = new Vector3(a.X.Z);
                result.X = x * bX + y * bY + z * b.Z;
            }

            {
                var x = new Vector3(a.Y.X);
                var y = new Vector3(a.Y.Y);
                var z = new Vector3(a.Y.Z);
                result.Y = x * bX + y * bY + z * b.Z;
            }

            {
                var x = new Vector3(a.Z.X);
                var y = new Vector3(a.Z.Y);
                var z = new Vector3(a.Z.Z);
                result.Z = x * bX + y * bY + z * b.Z;
            }


        }






    }
}