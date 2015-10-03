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

        public static void Transpose(ref MatrixSIMD m, out MatrixSIMD transposed)
        {
            transposed.X = new Vector4(m.X.X, m.Y.X, m.Z.X, m.W.X);
            transposed.Y = new Vector4(m.X.Y, m.Y.Y, m.Z.Y, m.W.Y);
            transposed.Z = new Vector4(m.X.Z, m.Y.Z, m.Z.Z, m.W.Z);
            transposed.W = new Vector4(m.X.W, m.Y.W, m.Z.W, m.W.W);
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
            //Compensating for the possibility that b is actually the same variable as result worsens the runtime of the function.
            //Instead, trust the user to use the other multiply function when necessary.

            //{
            //    var x = new Vector4(a.X.X);
            //    var y = new Vector4(a.X.Y);
            //    var z = new Vector4(a.X.Z);
            //    var w = new Vector4(a.X.W);
            //    result.X = (x * b.X + y * b.Y) + (z * b.Z + w * b.W);
            //}

            //{
            //    var x = new Vector4(a.Y.X);
            //    var y = new Vector4(a.Y.Y);
            //    var z = new Vector4(a.Y.Z);
            //    var w = new Vector4(a.Y.W);
            //    result.Y = (x * b.X + y * b.Y) + (z * b.Z + w * b.W);
            //}

            //{
            //    var x = new Vector4(a.Z.X);
            //    var y = new Vector4(a.Z.Y);
            //    var z = new Vector4(a.Z.Z);
            //    var w = new Vector4(a.Z.W);
            //    result.Z = (x * b.X + y * b.Y) + (z * b.Z + w * b.W);
            //}

            //{
            //    var x = new Vector4(a.W.X);
            //    var y = new Vector4(a.W.Y);
            //    var z = new Vector4(a.W.Z);
            //    var w = new Vector4(a.W.W);
            //    result.W = (x * b.X + y * b.Y) + (z * b.Z + w * b.W);
            //}

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

            //{
            //    var x = new Vector4(a.X.X);
            //    var y = new Vector4(a.X.Y);
            //    result.X = x * b.X + y * b.Y;
            //    var z = new Vector4(a.X.Z);
            //    var w = new Vector4(a.X.W);
            //    result.X += z * b.Z + w * b.W;
            //}

            //{
            //    var x = new Vector4(a.Y.X);
            //    var y = new Vector4(a.Y.Y);
            //    result.Y = x * b.X + y * b.Y;
            //    var z = new Vector4(a.Y.Z);
            //    var w = new Vector4(a.Y.W);
            //    result.Y += z * b.Z + w * b.W;
            //}

            //{
            //    var x = new Vector4(a.Z.X);
            //    var y = new Vector4(a.Z.Y);
            //    result.Z = x * b.X + y * b.Y;
            //    var z = new Vector4(a.Z.Z);
            //    var w = new Vector4(a.Z.W);
            //    result.Z += z * b.Z + w * b.W;
            //}

            //{
            //    var x = new Vector4(a.W.X);
            //    var y = new Vector4(a.W.Y);
            //    result.W = x * b.X + y * b.Y;
            //    var z = new Vector4(a.W.Z);
            //    var w = new Vector4(a.W.W);
            //    result.W += z * b.Z + w * b.W;
            //}

            //result.X = (new Vector4(a.X.X) * b.X + new Vector4(a.X.Y) * b.Y) + (new Vector4(a.X.Z) * b.Z + new Vector4(a.X.W) * b.W);
            //result.Y = (new Vector4(a.Y.X) * b.X + new Vector4(a.Y.Y) * b.Y) + (new Vector4(a.Y.Z) * b.Z + new Vector4(a.Y.W) * b.W);
            //result.Z = (new Vector4(a.Z.X) * b.X + new Vector4(a.Z.Y) * b.Y) + (new Vector4(a.Z.Z) * b.Z + new Vector4(a.Z.W) * b.W);
            //result.W = (new Vector4(a.W.X) * b.X + new Vector4(a.W.Y) * b.Y) + (new Vector4(a.W.Z) * b.Z + new Vector4(a.W.W) * b.W);

            //{
            //    var x = new Vector4(a.X.X);
            //    var y = new Vector4(a.X.Y);
            //    var z = new Vector4(a.X.Z);
            //    var w = new Vector4(a.X.W);
            //    result.X = x * b.X + y * b.Y + z * b.Z + w * b.W;
            //}

            //{
            //    var x = new Vector4(a.Y.X);
            //    var y = new Vector4(a.Y.Y);
            //    var z = new Vector4(a.Y.Z);
            //    var w = new Vector4(a.Y.W);
            //    result.Y = x * b.X + y * b.Y + z * b.Z + w * b.W;
            //}

            //{
            //    var x = new Vector4(a.Z.X);
            //    var y = new Vector4(a.Z.Y);
            //    var z = new Vector4(a.Z.Z);
            //    var w = new Vector4(a.Z.W);
            //    result.Z = x * b.X + y * b.Y + z * b.Z + w * b.W;
            //}

            //{
            //    var x = new Vector4(a.W.X);
            //    var y = new Vector4(a.W.Y);
            //    var z = new Vector4(a.W.Z);
            //    var w = new Vector4(a.W.W);
            //    result.W = x * b.X + y * b.Y + z * b.Z + w * b.W;
            //}


            //{
            //    var x = new Vector4(a.X.X);
            //    var y = new Vector4(a.X.Y);
            //    var z = new Vector4(a.X.Z);
            //    var w = new Vector4(a.X.W);
            //    var intermediateX = x * b.X;
            //    var intermediateY = y * b.Y;
            //    var intermediateXY = intermediateX + intermediateY;
            //    var intermediateZ = z * b.Z;
            //    var intermediateW = w * b.W;
            //    var intermediateZW = intermediateZ + intermediateW;
            //    result.X = intermediateXY + intermediateZW;
            //}

            //{
            //    var x = new Vector4(a.Y.X);
            //    var y = new Vector4(a.Y.Y);
            //    var z = new Vector4(a.Y.Z);
            //    var w = new Vector4(a.Y.W);
            //    var intermediateX = x * b.X;
            //    var intermediateY = y * b.Y;
            //    var intermediateXY = intermediateX + intermediateY;
            //    var intermediateZ = z * b.Z;
            //    var intermediateW = w * b.W;
            //    var intermediateZW = intermediateZ + intermediateW;
            //    result.Y = intermediateXY + intermediateZW;
            //}

            //{
            //    var x = new Vector4(a.Z.X);
            //    var y = new Vector4(a.Z.Y);
            //    var z = new Vector4(a.Z.Z);
            //    var w = new Vector4(a.Z.W);
            //    var intermediateX = x * b.X;
            //    var intermediateY = y * b.Y;
            //    var intermediateXY = intermediateX + intermediateY;
            //    var intermediateZ = z * b.Z;
            //    var intermediateW = w * b.W;
            //    var intermediateZW = intermediateZ + intermediateW;
            //    result.Z = intermediateXY + intermediateZW;
            //}

            //{
            //    var x = new Vector4(a.W.X);
            //    var y = new Vector4(a.W.Y);
            //    var z = new Vector4(a.W.Z);
            //    var w = new Vector4(a.W.W);
            //    var intermediateX = x * b.X;
            //    var intermediateY = y * b.Y;
            //    var intermediateXY = intermediateX + intermediateY;
            //    var intermediateZ = z * b.Z;
            //    var intermediateW = w * b.W;
            //    var intermediateZW = intermediateZ + intermediateW;
            //    result.W = intermediateXY + intermediateZW;
            //}
        }

    }
}
