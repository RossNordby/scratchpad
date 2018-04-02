using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Repro
{
    class BoundingBoxRepro
    {
        struct BoundingBoxTest
        {
            public Vector3 Min;
            public Vector3 Max;

            public override int GetHashCode()
            {
                return Min.GetHashCode() + Max.GetHashCode();
            }
        }

        public static void Corrupt()
        {
            var box = new BoundingBoxTest();
            box.Min = Vector3.Min(box.Min, box.Min);
            var hmm = box.GetHashCode();
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Test()
        {
            var someMemory = new int[1];
            var someMoreMemory = new int[1];
            Corrupt();
            someMoreMemory[someMemory[0]] = 0;
        }
    }
}
