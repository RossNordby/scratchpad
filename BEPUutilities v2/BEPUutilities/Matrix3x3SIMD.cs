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
        /// Calculates the determinant of the matrix.
        /// </summary>
        /// <returns>The matrix's determinant.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float Determinant()
        {
            //Current implementation of cross far from optimal without shuffles. This assumes it'll eventually be accelerated.
            Vector3 cross;
            Vector3x.Cross(ref Y, ref Z, out cross);
            return Vector3.Dot(X, cross);
        }

        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="m">Matrix to be inverted.</param>
        /// <param name="inverse">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(ref Matrix3x3SIMD m, out Matrix3x3SIMD inverse)
        {
            //Current implementation of cross far from optimal without shuffles, and even then this has some room for improvement.
            //Inverts should be really rare, so it's not too concerning. Use the scalar version when possible until ryujit improves (and we improve this implementation).
            Vector3 yz, zx, xy;
            Vector3x.Cross(ref m.Y, ref m.Z, out yz);
            Vector3x.Cross(ref m.Z, ref m.X, out zx);
            Vector3x.Cross(ref m.X, ref m.Y, out xy);
            var inverseDeterminant = 1f / Vector3.Dot(m.X, yz);
            inverse.X = yz * inverseDeterminant;
            inverse.Y = zx * inverseDeterminant;
            inverse.Z = xy * inverseDeterminant;
            Transpose(ref inverse, out inverse);
        }
        struct V
        {
            public float X, Y, Z;
        }


        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="m">Matrix to be inverted.</param>
        /// <param name="inverse">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static void Invert(Matrix3x3SIMD* m, Matrix3x3SIMD* inverse)
        {
            //var mScalar = (M*)m;
            //var inverseScalar = (M*)inverse;
            //Vector3 yz = new Vector3(
            //    mScalar->M22 * mScalar->M33 - mScalar->M32 * mScalar->M23,
            //    mScalar->M23 * mScalar->M31 - mScalar->M33 * mScalar->M21,
            //    mScalar->M21 * mScalar->M32 - mScalar->M31 * mScalar->M22);
            //var determinantInverse = new Vector3(1f / Vector3.Dot(m->X, yz));
            //inverse->X = yz * determinantInverse;

            //Vector3 zx = new Vector3(
            //    mScalar->M32 * mScalar->M13 - mScalar->M12 * mScalar->M33,
            //    mScalar->M33 * mScalar->M11 - mScalar->M13 * mScalar->M31,
            //    mScalar->M31 * mScalar->M12 - mScalar->M11 * mScalar->M32);
            //inverse->Y = zx * determinantInverse;

            //Vector3 xy = new Vector3(
            //    mScalar->M12 * mScalar->M23 - mScalar->M22 * mScalar->M13,
            //    mScalar->M13 * mScalar->M21 - mScalar->M23 * mScalar->M11,
            //    mScalar->M11 * mScalar->M22 - mScalar->M21 * mScalar->M12);
            //inverse->Z = xy * determinantInverse;

            //var m12 = inverseScalar->M12;
            //var m13 = inverseScalar->M13;
            //var m23 = inverseScalar->M23;
            //inverseScalar->M12 = inverseScalar->M21;
            //inverseScalar->M13 = inverseScalar->M31;
            //inverseScalar->M23 = inverseScalar->M32;
            //inverseScalar->M21 = m12;
            //inverseScalar->M31 = m13;
            //inverseScalar->M32 = m23;


            //var mScalar = (M*)m;
            //var inverseScalar = (M*)inverse;
            //Vector3 yz;
            //var yzPointer = (V*)&yz;
            //yzPointer->X = mScalar->M22 * mScalar->M33 - mScalar->M32 * mScalar->M23;
            //yzPointer->Y = mScalar->M23 * mScalar->M31 - mScalar->M33 * mScalar->M21;
            //yzPointer->Z = mScalar->M21 * mScalar->M32 - mScalar->M31 * mScalar->M22;
            //var determinantInverse = new Vector3(1f / Vector3.Dot(((Matrix3x3SIMD*)mScalar)->X, yz));
            //inverse->X = yz * determinantInverse;

            //Vector3 zx = new Vector3(
            //    mScalar->M32 * mScalar->M13 - mScalar->M12 * mScalar->M33,
            //    mScalar->M33 * mScalar->M11 - mScalar->M13 * mScalar->M31,
            //    mScalar->M31 * mScalar->M12 - mScalar->M11 * mScalar->M32);
            //inverse->Y = zx * determinantInverse;

            //Vector3 xy = new Vector3(
            //    mScalar->M12 * mScalar->M23 - mScalar->M22 * mScalar->M13,
            //    mScalar->M13 * mScalar->M21 - mScalar->M23 * mScalar->M11,
            //    mScalar->M11 * mScalar->M22 - mScalar->M21 * mScalar->M12);
            //inverse->Z = xy * determinantInverse;

            //var m12 = inverseScalar->M12;
            //var m13 = inverseScalar->M13;
            //var m23 = inverseScalar->M23;
            //inverseScalar->M12 = inverseScalar->M21;
            //inverseScalar->M13 = inverseScalar->M31;
            //inverseScalar->M23 = inverseScalar->M32;
            //inverseScalar->M21 = m12;
            //inverseScalar->M31 = m13;
            //inverseScalar->M32 = m23;

            //var mScalar = (M*)m;
            //var inverseScalar = (M*)inverse;
            //V c1;
            //c1.X = mScalar->M22 * mScalar->M33 - mScalar->M32 * mScalar->M23;
            //c1.Y = mScalar->M23 * mScalar->M31 - mScalar->M33 * mScalar->M21;
            //c1.Z = mScalar->M21 * mScalar->M32 - mScalar->M31 * mScalar->M22;
            //var determinantInverse = 1f / (c1.X * mScalar->M11 + c1.Y * mScalar->M12 + c1.Z * mScalar->M13);

            //V c2;
            //c2.X = mScalar->M32 * mScalar->M13 - mScalar->M12 * mScalar->M33;
            //c2.Y = mScalar->M33 * mScalar->M11 - mScalar->M13 * mScalar->M31;
            //c2.Z = mScalar->M31 * mScalar->M12 - mScalar->M11 * mScalar->M32;

            //V c3;
            //c3.X = mScalar->M12 * mScalar->M23 - mScalar->M22 * mScalar->M13;
            //c3.Y = mScalar->M13 * mScalar->M21 - mScalar->M23 * mScalar->M11;
            //c3.Z = mScalar->M11 * mScalar->M22 - mScalar->M21 * mScalar->M12;

            //inverseScalar->M11 = c1.X * determinantInverse;
            //inverseScalar->M21 = c1.Y * determinantInverse;
            //inverseScalar->M31 = c1.Z * determinantInverse;

            //inverseScalar->M12 = c2.X * determinantInverse;
            //inverseScalar->M22 = c2.Y * determinantInverse;
            //inverseScalar->M32 = c2.Z * determinantInverse;

            //inverseScalar->M13 = c3.X * determinantInverse;
            //inverseScalar->M23 = c3.Y * determinantInverse;
            //inverseScalar->M33 = c3.Z * determinantInverse;

            var mScalar = (M*)m;
            var inverseScalar = (M*)inverse;

            var m11 = mScalar->M22 * mScalar->M33 - mScalar->M32 * mScalar->M23;
            var m21 = mScalar->M23 * mScalar->M31 - mScalar->M33 * mScalar->M21;
            var m31 = mScalar->M21 * mScalar->M32 - mScalar->M31 * mScalar->M22;
            var determinantInverse = 1f / (m11 * mScalar->M11 + m21 * mScalar->M12 + m31 * mScalar->M13);

            var m12 = mScalar->M32 * mScalar->M13 - mScalar->M12 * mScalar->M33;
            var m22 = mScalar->M33 * mScalar->M11 - mScalar->M13 * mScalar->M31;
            var m32 = mScalar->M31 * mScalar->M12 - mScalar->M11 * mScalar->M32;

            var m13 = mScalar->M12 * mScalar->M23 - mScalar->M22 * mScalar->M13;
            var m23 = mScalar->M13 * mScalar->M21 - mScalar->M23 * mScalar->M11;
            var m33 = mScalar->M11 * mScalar->M22 - mScalar->M21 * mScalar->M12;

            inverseScalar->M11 = m11 * determinantInverse;
            inverseScalar->M21 = m21 * determinantInverse;
            inverseScalar->M31 = m31 * determinantInverse;

            inverseScalar->M12 = m12 * determinantInverse;
            inverseScalar->M22 = m22 * determinantInverse;
            inverseScalar->M32 = m32 * determinantInverse;

            inverseScalar->M13 = m13 * determinantInverse;
            inverseScalar->M23 = m23 * determinantInverse;
            inverseScalar->M33 = m33 * determinantInverse;
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