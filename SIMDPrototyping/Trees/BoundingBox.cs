using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees
{
    public struct BoundingBox
    {
        public Vector3 Min;
        public Vector3 Max;
    }



    [StructLayout(LayoutKind.Sequential]
    public struct BoundingBoxWidth4
    {
        [FieldOffset(0)]
        public Vector3Width4 Min;
        [FieldOffset(48)]
        public Vector3Width4 Max;

        [Flags]
        enum Mask
        {
            A = 1,
            B = 2,
            C = 4,
            D = 8
        }

        public BoundingBoxWidth4(ref BoundingBox a, ref BoundingBox b, ref BoundingBox c, ref BoundingBox d)
        {
            Min.X = new Vector4(a.Min.X, b.Min.X, c.Min.X, d.Min.X);
            Min.Y = new Vector4(a.Min.Y, b.Min.Y, c.Min.Y, d.Min.Y);
            Min.Z = new Vector4(a.Min.Z, b.Min.Z, c.Min.Z, d.Min.Z);
            Max.X = new Vector4(a.Max.X, b.Max.X, c.Max.X, d.Max.X);
            Max.Y = new Vector4(a.Max.Y, b.Max.Y, c.Max.Y, d.Max.Y);
            Max.Z = new Vector4(a.Max.Z, b.Max.Z, c.Max.Z, d.Max.Z);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Mask Intersects(ref BoundingBoxWidth4 a, ref BoundingBoxWidth4 b)
        {
            
        }
    }
}
