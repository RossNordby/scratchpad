using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests.SpecializedTests
{
    public class ExecutionEngineExceptionRepro
    {
        public struct Wrapper
        {
            public Vector3 Vector1;
            public Vector3 Vector2;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void SomeOtherFunction(ref Wrapper w)
        {
        }

        void SomeFunction()
        {
            var wrapper = new Wrapper();
            SomeOtherFunction(ref wrapper);
            var v = new Vector<float>();
        }
        
        public static int Test()
        {
            var test = new ExecutionEngineExceptionRepro();
            test.SomeFunction();
            return test.GetHashCode();
        }
    }
}
