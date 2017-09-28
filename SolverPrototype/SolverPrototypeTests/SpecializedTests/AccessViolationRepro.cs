using BEPUutilities2;
using BEPUutilities2.Memory;
using DemoRenderer;
using DemoUtilities;
using SolverPrototype;
using SolverPrototype.Collidables;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototypeTests.SpecializedTests
{
    public class AccessViolationRepro
    {
        //This reference will get corrupted, leading to a quick access violation.
        public object Reference = new object();

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

        public unsafe static int Test()
        {
            var test = new AccessViolationRepro();
            test.SomeFunction();
            return test.Reference.GetHashCode();
        }
    }

}
