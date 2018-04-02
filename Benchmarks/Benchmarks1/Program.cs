using BenchmarkDotNet.Running;

namespace Benchmarks1
{
    class Program
    {
        static void Main(string[] args)
        {
            BenchmarkRunner.Run<Pyramids>();
            BenchmarkRunner.Run<ShapePile>();
            BenchmarkRunner.Run<LotsOfStatics>();
        }
    }
}
