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
            //MicroFiddling.Cross();
            VectorizedConstraintTest.Test();
            HybridConstraintTest.Test();
            NewScalarConstraintTest.Test();
            OldScalarConstraintTest.Test();
            //Console.ReadKey();
        }
    }
}
