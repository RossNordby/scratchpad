using BEPUutilities2;
using SolverPrototype;
using SolverPrototype.CollisionDetection;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

using Quaternion = BEPUutilities2.Quaternion;

namespace SolverPrototypeTests
{
    public struct ConstraintlessLatticeBuilder : IConstraintBuilder
    {
        public void BuildConstraintsForBody(int sliceIndex, int rowIndex, int columnIndex,
            ref BodyDescription bodyDescription,
            ref LatticeBodyGetter ids,
            ref ConstraintAdder constraintAdder)
        {
        }

        public void RegisterConstraintTypes()
        {
        }
    }


}

