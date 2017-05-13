using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using static SolverPrototype.Solver;
using Quaternion = BEPUutilities2.Quaternion;

namespace SolverPrototypeTests
{
    static class ConstraintCacheOptimizationTests
    {
        public static void Test()
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);

            SimulationSetup.BuildLattice(32, 32, 32, out var simulation, out var bodyHandles, out var constraintHandles);

            SimulationSetup.ScrambleBodies(simulation);
            SimulationSetup.ScrambleConstraints(simulation.Solver);
            SimulationSetup.ScrambleBodyConstraintLists(simulation);
            SimulationSetup.AddRemoveChurn(simulation, 100000, bodyHandles, constraintHandles);

            var threadDispatcher = new SimpleThreadDispatcher(8);
            //var threadDispatcher = new NotQuiteAThreadDispatcher(8);

            const int bundlesPerOptimizationRegion = 1024;
            int constraintsPerOptimizationRegion = bundlesPerOptimizationRegion * Vector<int>.Count;
            int constraintOptimizationIterations = 8192;

            simulation.ConstraintLayoutOptimizer.Update(bundlesPerOptimizationRegion, simulation.BufferPool, threadDispatcher);//prejit
            var constraintsToOptimize = constraintsPerOptimizationRegion * constraintOptimizationIterations;
            //testOptimizer.Update(1, 1, simulation.BufferPool);
            var timer = Stopwatch.StartNew();
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                simulation.ConstraintLayoutOptimizer.Update(bundlesPerOptimizationRegion, simulation.BufferPool, threadDispatcher);
            }
            timer.Stop();
            Console.WriteLine($"Finished constraint optimizations, time (ms): {timer.Elapsed.TotalMilliseconds}" +
                $", per iteration (us): {timer.Elapsed.TotalSeconds * 1e6 / constraintOptimizationIterations}");


            for (int batchIndex = 0; batchIndex < simulation.Solver.Batches.Count; ++batchIndex)
            {
                var batch = simulation.Solver.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = (ContactManifold4TypeBatch)batch.TypeBatches[typeBatchIndex];
                    int[] sortKeys = new int[typeBatch.ConstraintCount];
                    int previous = -1;
                    //Console.WriteLine($"Batch {batchIndex}, type batch {typeBatchIndex}: ");
                    for (int i = 0; i < sortKeys.Length; ++i)
                    {
                        sortKeys[i] = ContactManifold4TypeBatch.GetSortKey(i, ref typeBatch.BodyReferences);
                        if (sortKeys[i] <= previous)
                        {
                            Console.WriteLine("Not sorted!");
                        }
                        previous = sortKeys[i];
                        //Console.Write($"{sortKeys[i]}, ");
                    }
                    //Console.WriteLine();
                }
            }

            //threadDispatcher.Dispose();
            simulation.BufferPool.Clear();

        }



    }
}

