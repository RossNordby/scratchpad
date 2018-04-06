using System;
using System.Numerics;

namespace SIMDWidthChecker
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine($"SIMD Width: {Vector<float>.Count}");
            Console.ReadKey();
        }
    }
}
