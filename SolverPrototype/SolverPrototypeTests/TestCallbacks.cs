using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
using SolverPrototype.Constraints;

namespace SolverPrototypeTests
{
    public struct TestCallbacks : INarrowPhaseCallbacks
    {
        public void Initialize(Simulation simulation)
        {
        }

        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            return true;
        }

        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, int childIndexA, int childIndexB)
        {
            return true;
        }

        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ref ContactManifold manifold, out PairMaterialProperties pairMaterial)
        {
            pairMaterial = new PairMaterialProperties();
            return false;
        }

        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ContactManifold manifold)
        {
            return false;
        }

        public void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandleA, int bodyHandleB, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
        }

        public void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandle, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
        }
        
        public void Flush(IThreadDispatcher threadDispatcher)
        {
        }

        public void Dispose()
        {
        }
    }
    
}
