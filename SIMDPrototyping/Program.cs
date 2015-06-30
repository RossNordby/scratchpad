using SIMDPrototyping.Tests;
using SIMDPrototyping.Trees.Tests;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    class Program
    {
        static Program()
        {
            Console.WriteLine($"Count: {Vector<int>.Count}");
        }
        static unsafe void Main(string[] args)
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
    }
}
