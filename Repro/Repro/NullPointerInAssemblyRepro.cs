using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Repro
{
    public static class NullPointerInAssemblyRepro
    {
        struct Container
        {
            public Vector<int> Vector;
            public int Integer;
        }

        static Vector<int> DoAThingByRef(ref Vector<int> s)
        {
            return s + Vector<int>.Zero;
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static unsafe Vector<int> Test()
        {
            Container container = default;
            return DoAThingByRef(ref container.Vector);
        }

        public static void TestRepro()
        {
            Test();
        }
    }
}
