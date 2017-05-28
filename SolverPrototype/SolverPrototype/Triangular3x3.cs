using BEPUutilities2;
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
    public struct Triangular3x3
    {
        /// <summary>
        /// First row, first column of the matrix.
        /// </summary>
        public float M11;
        /// <summary>
        /// Second row, first column of the matrix.
        /// </summary>
        public float M21;
        /// <summary>
        /// Second row, second column of the matrix.
        /// </summary>
        public float M22;
        /// <summary>
        /// Third row, first column of the matrix.
        /// </summary>
        public float M31;
        /// <summary>
        /// Third row, second column of the matrix.
        /// </summary>
        public float M32;
        /// <summary>
        /// Third row, third column of the matrix.
        /// </summary>
        public float M33;



    }
}
