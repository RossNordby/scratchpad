using SIMDPrototyping.Tests;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    class Program
    {
        static unsafe void Main(string[] args)
        {
            VectorAccelerationTest.Test();
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
