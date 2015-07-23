using SIMDPrototyping.Tests;
using SIMDPrototyping.Trees.Tests;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    class Program
    {

        static void Main2()
        {
            //Vector<int> ones = new Vector<int>(1);
            //Vector<int> fives = new Vector<int>(5);
            //int[] maskArray = new[] { 0, -1, 0, 0 };
            //var mask = new Vector<int>(maskArray);
            //var result = Vector.ConditionalSelect(mask, ones, fives);
            //Console.WriteLine($"result: {result}");
            Console.WriteLine("Vector hardware acceleration: " + Vector.IsHardwareAccelerated);

            TreeTest.Test();

            //VectorAccelerationTest.Test();
            //MicroFiddling.Cross();
            //VectorizedConstraintTest.Test();
            //VectorizedManifoldTest.Test();
            //HybridConstraintTest.Test();
            //NewScalarConstraintTest.Test();
            //SingleVectorizedConstraintTest.Test();
            //OldScalarConstraintTest.Test();
            //Console.ReadKey();
        }

        const int size = 16384;
        //[StructLayout(LayoutKind.Explicit, Size = size * sizeof(int))]
        //struct Stuff
        //{
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //unsafe static void Allocate2()
        //{
        //    //var pointer = new int[size];
        //    //Stuff stuff;
        //    //var pointer = (int*)&stuff;
        //    var pointer = stackalloc int[size];
        //    const int pleaseDontOptimizeMe = 5;
        //    for (int i = 0; i < size; i += size)
        //    {
        //        pointer[i] = pleaseDontOptimizeMe;
        //    }
        //}
        unsafe static void Allocate()
        {
            var pointer = stackalloc int[size];
            const int pleaseDontOptimizeMe = 5;
            //int fakeSize = (int)Math.Pow(size, 1);
            for (int i = 0; i < size; i+= size)
            {
                //var element = pointer + i;
                //*element = pleaseDontOptimizeMe;
                pointer[i] = pleaseDontOptimizeMe;
            }
        }

        unsafe static void Main()
        {
            var start = Stopwatch.GetTimestamp();
            for (int j = 0; j < 500000; ++j)
            {
                Allocate();
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"time: {(end - start) / (double)Stopwatch.Frequency}");
        }

    }
}
