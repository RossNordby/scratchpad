using System.Numerics;

namespace Repro
{
    class Program
    {
        static void Main(string[] args)
        {
            ExecutionEngineExceptionRepro.Test();
            BoundingBoxRepro.Test();
        }
    }
}
