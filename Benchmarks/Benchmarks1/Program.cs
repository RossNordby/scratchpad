using BenchmarkDotNet.Running;

namespace Benchmarks1
{
    class Program
    {
        static void Main(string[] args)
        {
            BenchmarkRunner.Run<Pyramids1>();
            BenchmarkRunner.Run<ShapePile1>();
            BenchmarkRunner.Run<LotsOfStatics1>();
            BenchmarkRunner.Run<ClothLattice1>();
        }
    }
}
