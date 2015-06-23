using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees
{
    /// <summary>
    /// 4-ary simd-friendly tree node.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 112)]
    public unsafe struct Node
    {
        [FieldOffset(0)]
        public Vector3Width4 Min;
        [FieldOffset(48)]
        public Vector3Width4 Max;
        [FieldOffset(96)]
        public fixed int Children[4];

    }
}
