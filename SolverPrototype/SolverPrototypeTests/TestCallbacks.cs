using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
using SolverPrototype.Constraints;
using System;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    public unsafe struct TestCallbacks : INarrowPhaseCallbacks
    {
        ImmediateConstraintAdder immediateAdder;
        public void Initialize(Simulation simulation)
        {
            immediateAdder.Initialize(simulation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 1;
            pairMaterial.MaximumRecoveryVelocity = 10.2f;
            pairMaterial.SpringSettings.NaturalFrequency = MathHelper.Pi * 30;
            pairMaterial.SpringSettings.DampingRatio = 100;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ContactManifold* manifold)
        {
            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandleA, int bodyHandleB, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
            immediateAdder.AddConstraint(workerIndex, constraintCacheIndex, ref impulses, bodyHandleA, bodyHandleB, ref constraintDescription);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddConstraint<TDescription>(int workerIndex, PairCacheIndex constraintCacheIndex, ref ContactImpulses impulses, int bodyHandle, ref TDescription constraintDescription) where TDescription : IConstraintDescription<TDescription>
        {
            immediateAdder.AddConstraint(workerIndex, constraintCacheIndex, ref impulses, bodyHandle, ref constraintDescription);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush(IThreadDispatcher threadDispatcher)
        {
        }

        public void Dispose()
        {
        }
    }

}
