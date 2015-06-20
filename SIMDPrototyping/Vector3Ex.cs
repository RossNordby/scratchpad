using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public static class Vector3Ex
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(ref Vector3 a, ref Vector3 b, out Vector3 result)
        {
            //{
            //    Vector3 shuffledA;
            //    shuffledA.X = a.Y;
            //    shuffledA.Y = a.Z;
            //    shuffledA.Z = a.X;
            //    Vector3 shuffledB;
            //    shuffledB.X = b.Y;
            //    shuffledB.Y = b.Z;
            //    shuffledB.Z = b.X;


            //    result = a * shuffledB - b * shuffledA;
            //    var temp = result.X;
            //    result.X = result.Y;
            //    result.Y = result.Z;
            //    result.Z = temp;
            //}

            //{
            //    var shuffledA = new Vector3(a.Y, a.Z, a.X);
            //    var shuffledB = new Vector3(b.Y, b.Z, b.X);

            //    result = a * shuffledB - b * shuffledA;
            //    result = new Vector3(result.Y, result.Z, result.X);
            //}

            //{
            //    var shuffledA = new Vector3(a.Y, a.Z, a.X);
            //    var shuffledB = new Vector3(b.Y, b.Z, b.X);

            //    var p = a * shuffledB - b * shuffledA;
            //    result = new Vector3(p.Y, p.Z, p.X);
            //}

            //{
            //    Vector3 shuffledA;
            //    shuffledA.X = a.Y;
            //    shuffledA.Y = a.Z;
            //    shuffledA.Z = a.X;
            //    Vector3 shuffledB;
            //    shuffledB.X = b.Y;
            //    shuffledB.Y = b.Z;
            //    shuffledB.Z = b.X;

            //    var p = a * shuffledB - b * shuffledA;
            //    result = new Vector3(p.Y, p.Z, p.X);
            //}

            float resultX = a.Y * b.Z - a.Z * b.Y;
            float resultY = a.Z * b.X - a.X * b.Z;
            float resultZ = a.X * b.Y - a.Y * b.X;
            result.X = resultX;
            result.Y = resultY;
            result.Z = resultZ;



        }
    }
}
