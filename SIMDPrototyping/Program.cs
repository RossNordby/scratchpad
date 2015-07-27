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

        static void Main()
        {
            Console.WriteLine("Vector hardware acceleration: " + Vector.IsHardwareAccelerated);

            TreeTest.Test();

            
        }


    }
}
