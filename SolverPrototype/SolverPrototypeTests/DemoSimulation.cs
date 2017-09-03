using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;

namespace SolverPrototypeTests
{
    public struct TestFilters : INarrowPhaseFilters
    {
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
            return true;
        }

        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ContactManifold manifold)
        {
            return true;
        }

        public void Flush(IThreadDispatcher threadDispatcher)
        {
        }

        public void Initialize<TNarrowPhase, TSupplementData>(Simulation<TNarrowPhase, TSupplementData> simulation)
            where TNarrowPhase : NarrowPhase, new()
            where TSupplementData : struct
        {
        }
    }

    public struct TestConstraintRemover : INarrowPhaseConstraintRemover
    {
        public void EnqueueConstraintRemoval(int workerIndex, int constraintHandle)
        {
        }

        public void Flush(IThreadDispatcher threadDispatcher)
        {
        }

        public void Initialize(Solver solver)
        {
        }
    }

    public class TestNarrowPhase : NarrowPhase<TestFilters, ImmediateConstraintAdder, TestConstraintRemover, int>
    { }

    /// <summary>
    /// Default simulation type used by most demos. You can pick and choose the features in your own simulation type.
    /// </summary>
    public class DemoSimulation : Simulation<TestNarrowPhase, int>
    {
        public DemoSimulation(BufferPool bufferPool, SimulationAllocationSizes initialAllocationSizes) : base(bufferPool, initialAllocationSizes)
        {
        }
        public DemoSimulation(BufferPool bufferPool) : base(bufferPool)
        {
        }
    }
}
