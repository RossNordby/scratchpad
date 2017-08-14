using System;
using System.Collections.Generic;
using System.Text;
using BEPUutilities2;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;

namespace SolverPrototype.CollisionDetection
{
    /// <summary>
    /// When notified of a new constraint, immediately adds it to the solver.
    /// </summary>
    public struct ImmediateConstraintAdder : INarrowPhaseConstraintAdder
    {
        Solver solver;

        public void Initialize(Solver solver)
        {
            throw new NotImplementedException();
        }

        public void AddConstraint<TDescription>(int workerIndex, TypedIndex constraintCacheIndex, int bodyHandleA, int bodyHandleB, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
            throw new NotImplementedException();
        }

        public void AddConstraint<TDescription>(int workerIndex, TypedIndex constraintCacheIndex, int bodyHandle, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
            throw new NotImplementedException();
        }

        public void Flush(IThreadDispatcher threadDispatcher)
        {
            throw new NotImplementedException();
        }

    }
}
