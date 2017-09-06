using System;
using System.Collections.Generic;
using System.Text;
using BEPUutilities2;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System.Runtime.CompilerServices;

namespace SolverPrototype.CollisionDetection
{
    /// <summary>
    /// When notified of a new constraint, immediately adds it to the solver.
    /// </summary>
    public struct ImmediateConstraintAdder<TNarrowPhase> where TNarrowPhase : NarrowPhase, new()
    {

        Simulation<TNarrowPhase> simulation;

        public void Initialize(Simulation<TNarrowPhase> simulation)
        {
            this.simulation = simulation;
        }

        public void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandleA, int bodyHandleB, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
            //Quick and dirty immediate adder. You will likely want to change this to a spinlock, and you may also want to go further and batch multiple additions together eventually.
            //The possibility of deferral is the reason we have to call back into the pair cache.
            int constraintHandle;
            lock (simulation.Solver)
            {
                constraintHandle = simulation.Add(bodyHandleA, bodyHandleB, ref constraintDescription);
            }
            //We have a guarantee that the pair cache will already have a slot allocated before this function is called.
            //TODO: The fact that the caller has to do some form of internal synchronization to add to the pair cache implies that the immediate adder could share it.
            //Could argue that the constraint add call should be invoked within the same lock with a guarantee of global sequential access.
            //This depends on the implementation of the pair cache. If it does subglobal locks, then 
            //(Notably, any ConstraintAdder implementation that defers adds could not share the lock, and extending the lock to cover AddConstraint would cause overhead.)
            simulation.NarrowPhase.PairCache.CompleteConstraintAdd(simulation.Solver, ref impulses, constraintCacheIndex, constraintHandle);
        }
        
        public void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandle, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
            throw new NotImplementedException();
        }

        public void Flush(IThreadDispatcher threadDispatcher)
        {
        }

    }
}
