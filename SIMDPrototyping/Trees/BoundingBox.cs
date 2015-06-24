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


    [StructLayout(LayoutKind.Sequential)]
    public struct BoundingBoxWide
    {
        public Vector3Wide Min;
        public Vector3Wide Max;


        public BoundingBoxWide(ref BoundingBox boundingBox)
        {
            Min = new Vector3Wide(ref boundingBox.Min);
            Max = new Vector3Wide(ref boundingBox.Max);
        }

    
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static Vector<int> Intersects(ref BoundingBoxWide a, ref BoundingBoxWide b)
        {
            //If any minimum exceeds the other maximum, there can be no collision.
            //On the flipside, if the all minimums are less than the opposing maximums, then they must be colliding.
            var c1X = Vector.LessThanOrEqual(a.Min.X, b.Max.X);
            var c1Y = Vector.LessThanOrEqual(a.Min.Y, b.Max.Y);
            var c1Z = Vector.LessThanOrEqual(a.Min.Z, b.Max.Z);
            var c2X = Vector.LessThanOrEqual(b.Min.X, a.Max.X);
            var c2Y = Vector.LessThanOrEqual(b.Min.Y, a.Max.Y);
            var c2Z = Vector.LessThanOrEqual(b.Min.Z, a.Max.Z);
            var or1 = Vector.BitwiseOr(c1X, c1Y);
            var or2 = Vector.BitwiseOr(c1Z, c2X);
            var or3 = Vector.BitwiseOr(c2Y, c2Z);
            var or4 = Vector.BitwiseOr(or1, or2);
            return Vector.BitwiseOr(or3, or4);


        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static Vector<int> Intersects2(ref BoundingBoxWide a, ref BoundingBoxWide b)
        {
            
            var minX = Vector.Max(a.Min.X, b.Min.X);
            var minY = Vector.Max(a.Min.Y, b.Min.Y);
            var minZ = Vector.Max(a.Min.Z, b.Min.Z);
            var maxX = Vector.Min(a.Max.X, b.Max.X);
            var maxY = Vector.Min(a.Max.Y, b.Max.Y);
            var maxZ = Vector.Min(a.Max.Z, b.Max.Z);
            var xLeq = Vector.LessThanOrEqual(minX, maxX);
            var yLeq = Vector.LessThanOrEqual(minX, maxX);
            var zLeq = Vector.LessThanOrEqual(minX, maxX);
            return Vector.BitwiseAnd(xLeq, Vector.BitwiseAnd(yLeq, zLeq));


        }

        void Merge(ref BoundingBoxWide a, ref BoundingBoxWide b, out BoundingBoxWide merged)
        {
            merged = new BoundingBoxWide();
        }

        void ComputeVolume(ref BoundingBoxWide boxes, out Vector<float> volumes)
        {
            volumes = new Vector<float>();
            dot proeductsse
        }
    }
}
