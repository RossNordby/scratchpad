﻿using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace SolverPrototype
{
    struct ConstraintGraphRemovalEnumerator : IForEach<int>
    {
        internal ConstraintConnectivityGraph graph;
        internal int constraintHandle;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void LoopBody(int bodyIndex)
        {
            graph.RemoveConstraint(bodyIndex, constraintHandle);
        }
    }
}
