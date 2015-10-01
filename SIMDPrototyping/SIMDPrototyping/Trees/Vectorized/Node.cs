using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees.Vectorized
{
    /// <summary>
    /// 4-wide simd-friendly tree node.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public unsafe struct Node
    {
        public BoundingBoxWide BoundingBoxes;
        public Vector<int> Children;
    }
}
