using BenchmarkDotNet.Running;
using System;

namespace Benchmarks2
{
    class Program
    {
        static void Main(string[] args)
        {
            BenchmarkRunner.Run<Pyramids>();
            BenchmarkRunner.Run<ShapePile>();
            BenchmarkRunner.Run<LotsOfStatics>();
            BenchmarkRunner.Run<ClothLattice>();
        }
    }
}
