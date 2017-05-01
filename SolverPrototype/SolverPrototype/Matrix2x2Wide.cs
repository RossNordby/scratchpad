using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{

    public struct Matrix2x2Wide
    {
        //TODO: While it would be a little more convenient to store these rows as Vector2Wides, there is a codegen issue with nested structs as of Microsoft.NETCore.App 2.0.0-preview2-002093-00.
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
        /// Value at row 2, column 1 of the matrix.
        /// </summary>
        public Vector<float> M21;
        /// <summary>
        /// Value at row 2, column 2 of the matrix.
        /// </summary>
        public Vector<float> M22;

        //These helpers attempt to make up for the convenience lost from the row flattening.
        /// <summary>
        /// Gets a reference to the first row of the matrix.
        /// </summary>
        /// <param name="m">Matrix to get the row from.</param>
        /// <returns>Reference to the first row of the matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector2Wide GetX(ref Matrix2x2Wide m)
        {
            return ref Unsafe.As<Vector<float>, Vector2Wide>(ref m.M11);
        }
        /// <summary>
        /// Gets a reference to the second row of the matrix.
        /// </summary>
        /// <param name="m">Matrix to get the row from.</param>
        /// <returns>Reference to the second row of the matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector2Wide GetY(ref Matrix2x2Wide m)
        {
            return ref Unsafe.As<Vector<float>, Vector2Wide>(ref m.M21);
        }


        /// <summary>
        /// Multiplies a matrix by another matrix, where the second matrix is sampled as if it were transposed: result = a * transpose(b).
        /// </summary>
        /// <param name="a">First matrix in the pair.</param>
        /// <param name="b">Matrix to be sampled as if it were transposed when multiplied with the first matrix.</param>
        /// <param name="result">Result of the multiplication a * transpose(b).</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyByTransposeWithoutOverlap(ref Matrix2x2Wide a, ref Matrix2x2Wide b, out Matrix2x2Wide result)
        {
            result.M11 = a.M11 * b.M11 + a.M12 * b.M12;
            result.M12 = a.M11 * b.M21 + a.M12 * b.M22;
            result.M21 = a.M21 * b.M11 + a.M22 * b.M12;
            result.M22 = a.M21 * b.M21 + a.M22 * b.M22;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(ref Vector2Wide v, ref Matrix2x2Wide m, out Vector2Wide result)
        {
            result.X = v.X * m.M11 + v.Y * m.M21;
            result.Y = v.X * m.M12 + v.Y * m.M22;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector2Wide v, ref Matrix2x2Wide m, out Vector2Wide result)
        {
            TransformWithoutOverlap(ref v, ref m, out var temp);
            result = temp;
        }


        /// <summary>
        /// Multiplies every component in the matrix by the given scalar value.
        /// </summary>
        /// <param name="m">Matrix to scale.</param>
        /// <param name="scale">Scaling value to apply to the matrix's components.</param>
        /// <param name="result">Resulting matrix with scaled components.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Matrix2x2Wide m, ref Vector<float> scale, out Matrix2x2Wide result)
        {
            result.M11 = m.M11 * scale;
            result.M12 = m.M12 * scale;
            result.M21 = m.M21 * scale;
            result.M22 = m.M22 * scale;
        }

        /// <summary>
        /// Adds the components of one matrix to another.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two given matrices.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Matrix2x2Wide a, ref Matrix2x2Wide b, out Matrix2x2Wide result)
        {
            result.M11 = a.M11 + b.M11;
            result.M12 = a.M12 + b.M12;
            result.M21 = a.M21 + b.M21;
            result.M22 = a.M22 + b.M22;
        }

        /// <summary>
        /// Subtracts the components of one matrix from another.
        /// </summary>
        /// <param name="a">Matrix to be subtracted from..</param>
        /// <param name="b">Matrix to subtract from the other.</param>
        /// <param name="result">Result of the subtraction.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Subtract(ref Matrix2x2Wide a, ref Matrix2x2Wide b, out Matrix2x2Wide result)
        {
            result.M11 = a.M11 - b.M11;
            result.M12 = a.M12 - b.M12;
            result.M21 = a.M21 - b.M21;
            result.M22 = a.M22 - b.M22;
        }



        /// <summary>
        /// Inverts the given matix.
        /// </summary>
        /// <param name="matrix">Matrix to be inverted.</param>
        /// <param name="result">Inverted matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InvertWithoutOverlap(ref Matrix2x2Wide m, out Matrix2x2Wide inverse)
        {
            var determinantInverse = Vector<float>.One / (m.M11 * m.M22 - m.M12 * m.M21);
            inverse.M11 = m.M22 * determinantInverse;
            inverse.M12 = -m.M12 * determinantInverse;

            inverse.M21 = -m.M21 * determinantInverse;
            inverse.M22 = m.M11 * determinantInverse;

        }
    }
}
