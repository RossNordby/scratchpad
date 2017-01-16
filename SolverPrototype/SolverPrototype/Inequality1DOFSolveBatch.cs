using BEPUutilities2.ResourceManagement;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Handles the solve iterations of a bunch of 1DOF two body inequality constraints.
    /// </summary>
    public class Inequality1DOFSolveBatch : SolveBatch<IterationData2Body1DOF>
    {
        public override void WarmStart(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            for (int i = startBundle; i < endBundle; ++i)
            {
                Inequality2Body1DOF.WarmStart(bodyVelocities, ref IterationData[i]);
            }
        }
        public override void SolveIteration(BodyVelocities[] bodyVelocities, int startBundle, int endBundle)
        {
            for (int i = startBundle; i < endBundle; ++i)
            {
                Inequality2Body1DOF.Solve(bodyVelocities, ref IterationData[i]);
            }
        }

    }
}
