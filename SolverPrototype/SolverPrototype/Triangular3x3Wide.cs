using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototype
{
    /// <summary>
    /// Stores the lower left triangle (including diagonal) of a 3x3 matrix. Useful for triangular forms and (anti)symmetric matrices.
    /// </summary>
    public struct Triangular3x3Wide
    {
        /// <summary>
        /// First row, first column of the matrix.
        /// </summary>
        public Vector<float> M11;
        /// <summary>
        /// Second row, first column of the matrix.
        /// </summary>
        public Vector<float> M21;
        /// <summary>
        /// Second row, second column of the matrix.
        /// </summary>
        public Vector<float> M22;
        /// <summary>
        /// Third row, first column of the matrix.
        /// </summary>
        public Vector<float> M31;
        /// <summary>
        /// Third row, second column of the matrix.
        /// </summary>
        public Vector<float> M32;
        /// <summary>
        /// Third row, third column of the matrix.
        /// </summary>
        public Vector<float> M33;

        /// <summary>
        /// Inverts the matrix as if it is a symmetric matrix where M32 == M23, M13 == M31, and M21 == M12.
        /// </summary>
        /// <param name="m">Symmetric matrix to invert.</param>
        /// <param name="inverse">Inverse of the symmetric matrix.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SymmetricInvert(ref Triangular3x3Wide m, out Triangular3x3Wide inverse)
        {
            var M11M22 = m.M11 * m.M22;
            var M21M31 = m.M21 * m.M31;
            var M21M21 = m.M21 * m.M21;
            var M31M31 = m.M31 * m.M31;
            var M32M32 = m.M32 * m.M32;
            var M21M31M32 = M21M31 * m.M32;
            var denom = Vector<float>.One / ((m.M22 * M31M31 - M21M31M32) + (m.M11 * M32M32 - M21M31M32) + (M21M21 - M11M22) * m.M33);
            var M22M33 = m.M22 * m.M33;
            var M31M32 = m.M31 * m.M32;
            var M21M33 = m.M21 * m.M33;
            var M11M33 = m.M11 * m.M33;
            var M22M31 = m.M22 * m.M31;
            var M21M32 = m.M21 * m.M32;
            var M11M32 = m.M11 * m.M32;
            inverse.M11 = (M32M32 - M22M33) * denom;
            inverse.M21 = (M21M33 - M31M32) * denom;
            inverse.M22 = (M31M31 - M11M33) * denom;
            inverse.M31 = (M22M31 - M21M32) * denom;
            inverse.M32 = (M11M32 - M21M31) * denom;
            inverse.M33 = (M21M21 - M11M22) * denom;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Triangular3x3Wide a, ref Triangular3x3Wide b, out Triangular3x3Wide sum)
        {
            sum.M11 = a.M11 + b.M11;
            sum.M21 = a.M21 + b.M21;
            sum.M22 = a.M22 + b.M22;
            sum.M31 = a.M31 + b.M31;
            sum.M32 = a.M32 + b.M32;
            sum.M33 = a.M33 + b.M33;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Scale(ref Triangular3x3Wide m, ref Vector<float> scale, out Triangular3x3Wide result)
        {
            result.M11 = m.M11 * scale;
            result.M21 = m.M21 * scale;
            result.M22 = m.M22 * scale;
            result.M31 = m.M31 * scale;
            result.M32 = m.M32 * scale;
            result.M33 = m.M33 * scale;
        }

        //If you ever need a triangular invert, a couple of options:
        //For matrices of the form:
        //[ 1   0   0 ]
        //[ M21 1   0 ]
        //[ M31 M32 1 ]
        //The inverse is simply:
        //       [ 1                0     0 ]
        //M^-1 = [ -M21             1     0 ]
        //       [ M21 * M32 - M31  -M32  1 ]

        //For a matrix with an arbitrary diagonal (that's still invertible):
        //       [ 1/M11                               0               0     ]
        //M^-1 = [ -M21/(M11*M22)                      1/M22           0     ]
        //       [ -(M22*M31 - M21*M32)/(M11*M22*M33)  -M32/(M22*M33)  1/M33 ]
        //And with some refiddling, you could make all the denominators the same to avoid repeated divisions.

        /// <summary>
        /// Computes skewSymmetric(v) * m * transpose(skewSymmetric(v)) for a symmetric matrix m. Assumes that the input and output matrices do not overlap.
        /// </summary>
        /// <param name="m">Symmetric matrix.</param>
        /// <param name="v">Vector to create the skew symmetric matrix from.</param>
        /// <param name="sandwich">Result of skewSymmetric(v) * m * transpose(skewSymmetric(v)).</param>
        /// <remarks>This operation might have a formal name that isn't skew sandwich. But that's okay, its real name is skew sandwich.</remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SkewSandwichWithoutOverlap(ref Vector3Wide v, ref Triangular3x3Wide m, out Triangular3x3Wide sandwich)
        {
            var vxvx = v.X * v.X;
            var vyvy = v.Y * v.Y;
            var vzvz = v.Z * v.Z;
            var vxvy = v.X * v.Y;
            var vxvz = v.X * v.Z;
            var vyvz = v.Y * v.Z;
            var M32vyvz = m.M32 * vyvz;
            var M31vxvz = m.M31 * vxvz;
            var M21vxvy = m.M21 * vxvy;
            sandwich.M11 = (m.M33 * vyvy - M32vyvz) + (m.M22 * vzvz - M32vyvz);
            sandwich.M21 = -m.M33 * vxvy + m.M32 * vxvz + m.M31 * vyvz - m.M21 * vzvz;
            sandwich.M22 = (m.M33 * vxvx - M31vxvz) + (m.M11 * vzvz - M31vxvz);
            sandwich.M31 = m.M32 * vxvy - m.M31 * vyvy - m.M22 * vxvz + m.M21 * vyvz;
            sandwich.M32 = -m.M32 * vxvx + m.M31 * vxvy + m.M21 * vxvz - m.M11 * vyvz;
            sandwich.M33 = (m.M22 * vxvx - M21vxvy) + (m.M11 * vyvy - M21vxvy);            
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformWithoutOverlap(ref Vector3Wide v, ref Triangular3x3Wide m, out Vector3Wide result)
        {
            result.X = v.X * m.M11 + v.Y * m.M21 + v.Z * m.M31;
            result.Y = v.X * m.M21 + v.Y * m.M22 + v.Z * m.M32;
            result.Z = v.X * m.M31 + v.Y * m.M32 + v.Z * m.M33;
        }

    }
}
