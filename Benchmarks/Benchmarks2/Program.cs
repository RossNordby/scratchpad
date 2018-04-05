using BenchmarkDotNet.Running;
using System;

namespace Benchmarks2
{
    class Program
    {
        static void Main(string[] args)
        {
            BenchmarkRunner.Run<Pyramids2>();
            BenchmarkRunner.Run<ShapePile2>();
            BenchmarkRunner.Run<LotsOfStatics2>();
            BenchmarkRunner.Run<ClothLattice2>();
        }
    }
}
