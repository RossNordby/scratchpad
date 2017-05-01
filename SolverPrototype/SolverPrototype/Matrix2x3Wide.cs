using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{

    public struct Matrix2x3Wide
    {
        //TODO: While it would be a little more convenient to store these rows as Vector3Wides, there is a codegen issue with nested structs as of Microsoft.NETCore.App 2.0.0-preview2-002093-00.
        //The jit emits zeroing for temporaries for nested structs, but not flattened ones.

        /// <summary>
        /// Value at row 1, column 1 of the matrix.
        /// </summary>
        public Vector<float> M11;
        /// <summary>
        /// Value at row 1, column 2 of the matrix.
        /// </summary>
        public Vector<float> M12;
        /// <summary>
        /// Value at row 1, column 3 of the matrix.
        /// </summary>
        public Vector<float> M13;
        /// <summary>
        /// Value at row 2, column 1 of the matrix.
        /// </summary>
        public Vector<float> M21;
        /// <summary>
        /// Value at row 2, column 2 of the matrix.
        /// </summary>
        public Vector<float> M22;
        /// <summary>
        /// Value at row 2, column 3 of the matrix.
        /// </summary>
        public Vector<float> M23;

        //These helpers attempt to make up for the convenience lost from the row flattening.
        /// <summary>
        /// Gets a reference to the first row of the matrix.
        /// </summary>
        /// <param name="m">Matrix to get the row from.</param>
        /// <returns>Reference to the first row of the matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide GetX(ref Matrix2x3Wide m)
        {
            return ref Unsafe.As<Vector<float>, Vector3Wide>(ref m.M11);
        }
        /// <summary>
        /// Gets a reference to the second row of the matrix.
        /// </summary>
        /// <param name="m">Matrix to get the row from.</param>
        /// <returns>Reference to the second row of the matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide GetY(ref Matrix2x3Wide m)
        {
            return ref Unsafe.As<Vector<float>, Vector3Wide>(ref m.M21);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(ref Matrix2x3Wide a, ref Matrix3x3Wide b, out Matrix2x3Wide result)
        {
            result.M11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31;
            result.M12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32;
            result.M13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33;
            result.M21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31;
            result.M22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32;
            result.M23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(ref Matrix2x2Wide a, ref Matrix2x3Wide b, out Matrix2x3Wide result)
        {
            result.M11 = a.M11 * b.M11 + a.M12 * b.M21;
            result.M12 = a.M11 * b.M12 + a.M12 * b.M22;
            result.M13 = a.M11 * b.M13 + a.M12 * b.M23;
            result.M21 = a.M21 * b.M11 + a.M22 * b.M21;
            result.M22 = a.M21 * b.M12 + a.M22 * b.M22;
            result.M23 = a.M21 * b.M13 + a.M22 * b.M23;
        }

        /// <summary>
        /// Multiplies a matrix by another matrix, where the first matrix is sampled as if it were transposed: result = transpose(a) * b.
        /// </summary>
        /// <param name="a">Matrix to be sampled as if it were transposed when multiplied with the second matrix.</param>
        /// <param name="b">Second matrix in the pair.</param>
        /// <param name="result">Result of the multiplication transpose(a) * b.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyTransposedWithoutOverlap(ref Matrix2x2Wide a, ref Matrix2x3Wide b, out Matrix2x3Wide result)
        {
            result.M11 = a.M11 * b.M11 + a.M21 * b.M21;
            result.M12 = a.M11 * b.M12 + a.M21 * b.M22;
            result.M13 = a.M11 * b.M13 + a.M21 * b.M23;
            result.M21 = a.M12 * b.M11 + a.M22 * b.M21;
            result.M22 = a.M12 * b.M12 + a.M22 * b.M22;
            result.M23 = a.M12 * b.M13 + a.M22 * b.M23;

        }

        /// <summary>
        /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
        /// </summary>
        /// <param name="a">First matrix in the pair.</param>
        /// <param name="b">Matrix to be sampled as if it were transposed when multiplied with the first matrix.</param>
        /// <param name="result">Result of the multiplication a * transpose(b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyByTransposeWithoutOverlap(ref Matrix2x3Wide a, ref Matrix2x3Wide b, out Matrix2x2Wide result)
        {
            result.M11 = a.M11 * b.M11 + a.M12 * b.M12 + a.M13 * b.M13;
            result.M12 = a.M11 * b.M21 + a.M12 * b.M22 + a.M13 * b.M23;
            result.M21 = a.M21 * b.M11 + a.M22 * b.M12 + a.M23 * b.M13;
            result.M22 = a.M21 * b.M21 + a.M22 * b.M22 + a.M23 * b.M23;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformByTransposeWithoutOverlap(ref Vector3Wide v, ref Matrix2x3Wide m, out Vector2Wide result)
        {
            result.X = v.X * m.M11 + v.Y * m.M12 + v.Z * m.M13;
            result.Y = v.X * m.M21 + v.Y * m.M22 + v.Z * m.M23;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Negate(ref Matrix2x3Wide m, out Matrix2x3Wide result)
        {
            result.M11 = -m.M11;
            result.M12 = -m.M12;
            result.M13 = -m.M13;
            result.M21 = -m.M21;
            result.M22 = -m.M22;
            result.M23 = -m.M23;
        }

        /// <summary>
        /// Multiplies every component in the matrix by the given scalar value.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scaling value to apply to the matrix's components.</param>
        /// <param name="result">Resulting matrix with scaled components.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Matrix2x3Wide m, ref Vector<float> scale, out Matrix2x3Wide result)
        {
            result.M11 = m.M11 * scale;
            result.M12 = m.M12 * scale;
            result.M13 = m.M13 * scale;
            result.M21 = m.M21 * scale;
            result.M22 = m.M22 * scale;
            result.M23 = m.M23 * scale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector2Wide v, ref Matrix2x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.M11 + v.Y * m.M21;
            result.Y = v.X * m.M12 + v.Y * m.M22;
            result.Z = v.X * m.M13 + v.Y * m.M23;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Matrix2x3Wide a, ref Matrix2x3Wide b, out Matrix2x3Wide result)
        {
            result.M11 = a.M11 + b.M11;
            result.M12 = a.M12 + b.M12;
            result.M13 = a.M13 + b.M13;
            result.M21 = a.M21 + b.M21;
            result.M22 = a.M22 + b.M22;
            result.M23 = a.M23 + b.M23;
        }
    }
}
