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
    static class ContactManifoldConvergenceTests
    {
        public static void Test()
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);

            SimulationSetup.BuildLattice(32, 32, 32, out var simulation, out var bodyHandles, out var constraintHandles);

            SimulationSetup.ScrambleBodies(simulation);
            SimulationSetup.ScrambleConstraints(simulation.Solver);
            SimulationSetup.ScrambleBodyConstraintLists(simulation);
            //SimulationSetup.AddRemoveChurn(simulation, 100000, bodyHandles, constraintHandles);

            //var threadPool = new TPLPool(8);
            var threadPool = new SimpleThreadDispatcher(8);

            double compressionTimeAccumulator = 0;
            const int iterations = 1;
            const int internalCompressionIterations = 1;
            //for (int i = 0; i < iterations; ++i)
            //{
            //    SimulationSetup.AddRemoveChurn(simulation, 100, bodyHandles, constraintHandles);
            //    GC.Collect(3, GCCollectionMode.Forced, true);
            //    var start = Stopwatch.GetTimestamp();
            //    for (int j = 0; j < internalCompressionIterations; ++j)
            //    {
            //        simulation.SolverBatchCompressor.Compress(simulation.BufferPool);
            //    }
            //    compressionTimeAccumulator += (Stopwatch.GetTimestamp() - start) / (double)Stopwatch.Frequency;
            //}
            Console.WriteLine($"Time per compression: {1e6 * compressionTimeAccumulator / (iterations * internalCompressionIterations)} us");
            GC.Collect(3, GCCollectionMode.Forced, true);

            //Attempt cache optimization.
            int bodyOptimizationIterations = bodyHandles.Length * 1;
            //bodyOptimizer.PartialIslandOptimizeDFS(bodyHandles.Length); //prejit
            //simulation.BodyLayoutOptimizer.DumbIncrementalOptimize(); //prejit
            var timer = Stopwatch.StartNew();
            for (int i = 0; i < bodyOptimizationIterations; ++i)
            {
                //bodyOptimizer.PartialIslandOptimizeDFS(64);
                //simulation.BodyLayoutOptimizer.DumbIncrementalOptimize();
                //simulation.BodyLayoutOptimizer.IncrementalOptimize(1, threadPool, simulation.BufferPool);
            }
            timer.Stop();
            var optimizationTime = timer.Elapsed.TotalSeconds;
            Console.WriteLine($"Finished {bodyOptimizationIterations} body optimizations, time (ms): {optimizationTime * 1e3}, per iteration (us): {optimizationTime * 1e6 / bodyOptimizationIterations}");

            //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
            int constraintCount = 0;
            for (int i = 0; i < simulation.Solver.Batches.Count; ++i)
            {
                for (int j = 0; j < simulation.Solver.Batches[i].TypeBatches.Count; ++j)
                {
                    constraintCount += simulation.Solver.Batches[i].TypeBatches[j].ConstraintCount;
                }
            }
            const int bundlesPerOptimizationRegion = 256;
            int constraintsPerOptimizationRegion = bundlesPerOptimizationRegion * Vector<int>.Count;
            const int regionsPerConstraintOptimizationIteration = 8;
            int constraintOptimizationIterations = 32768;
            //int constraintOptimizationIterations = Math.Max(16,
            //    (int)(1 * 2 * ((long)constraintCount * constraintCount /
            //    ((double)constraintsPerOptimizationRegion * constraintsPerOptimizationRegion)) / regionsPerConstraintOptimizationIteration));

            var testOptimizer = new ConstraintLayoutOptimizer2(simulation.Bodies, simulation.ConstraintGraph, simulation.Solver);
            //simulation.ConstraintLayoutOptimizer.Update(bundlesPerOptimizationRegion, regionsPerConstraintOptimizationIteration, simulation.BufferPool, threadPool);//prejit
            var constraintsToOptimize = constraintsPerOptimizationRegion * regionsPerConstraintOptimizationIteration * constraintOptimizationIterations;
            //testOptimizer.Update(1, 1, simulation.BufferPool);
            timer.Restart();
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                testOptimizer.Update(1, 4096, simulation.BufferPool);
                //simulation.ConstraintLayoutOptimizer.Update(bundlesPerOptimizationRegion, regionsPerConstraintOptimizationIteration, simulation.BufferPool, threadPool);

            }
            timer.Stop();
            Console.WriteLine($"Finished constraint optimizations, time (ms): {timer.Elapsed.TotalMilliseconds}" +
                $", per iteration (us): {timer.Elapsed.TotalSeconds * 1e6 / constraintOptimizationIterations}");
            return;

            //for (int batchIndex= 0; batchIndex < simulation.Solver.Batches.Count; ++batchIndex)
            //{
            //    var batch = simulation.Solver.Batches[batchIndex];
            //    for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
            //    {
            //        var typeBatch = (ContactManifold4TypeBatch)batch.TypeBatches[typeBatchIndex];
            //        int[] sortKeys = new int[typeBatch.ConstraintCount];
            //        int previous = -1;
            //        Console.WriteLine($"Batch {batchIndex}, type batch {typeBatchIndex}: ");
            //        for (int i = 0; i < sortKeys.Length; ++i)
            //        {
            //            sortKeys[i] = typeBatch.GetSortKey(i);
            //            if(sortKeys[i] <= previous)
            //            {
            //                Console.WriteLine("Not sorted!");
            //            }
            //            previous = sortKeys[i];
            //            Console.Write($"{sortKeys[i]}, ");
            //        }
            //        Console.WriteLine();
            //    }
            //}


            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 32;
            const int frameCount = 128;
            simulation.Solver.IterationCount = iterationCount;


            //prejit
            simulation.Solver.Update(dt, inverseDt);
            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            timer.Reset();

            //var threadPool = new NotQuiteAThreadPool();
            Console.WriteLine($"Using {threadPool.ThreadCount} workers.");
            double solveTime = 0;
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = simulation.Bodies.GetBodyEnergyHeuristic();
                //Update the penetration depths associated with the constraints.
                //This simulates actual position integration and repeated contact detection, allowing the constraints to properly spring.
                for (int i = 0; i < constraintHandles.Length; ++i)
                {
                    simulation.Solver.GetConstraintReference(constraintHandles[i], out var constraint);
                    var typeBatch = constraint.TypeBatch as ContactManifold4TypeBatch;

                    BundleIndexing.GetBundleIndices(constraint.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
                    typeBatch.BodyReferences[bundleIndex].Unpack(bundleIndex, typeBatch.ConstraintCount, out var bodyReferences);
                    var bodyBundleIndexA = GatherScatter.Get(ref bodyReferences.BundleIndexA, innerIndex);
                    var bodyBundleIndexB = GatherScatter.Get(ref bodyReferences.BundleIndexB, innerIndex);
                    var innerIndexA = GatherScatter.Get(ref bodyReferences.InnerIndexA, innerIndex);
                    var innerIndexB = GatherScatter.Get(ref bodyReferences.InnerIndexB, innerIndex);

                    var velocityA = new Vector3(
                        GatherScatter.Get(ref simulation.Bodies.VelocityBundles[bodyBundleIndexA].LinearVelocity.X, innerIndexA),
                        GatherScatter.Get(ref simulation.Bodies.VelocityBundles[bodyBundleIndexA].LinearVelocity.Y, innerIndexA),
                        GatherScatter.Get(ref simulation.Bodies.VelocityBundles[bodyBundleIndexA].LinearVelocity.Z, innerIndexA));
                    var velocityB = new Vector3(
                        GatherScatter.Get(ref simulation.Bodies.VelocityBundles[bodyBundleIndexB].LinearVelocity.X, innerIndexB),
                        GatherScatter.Get(ref simulation.Bodies.VelocityBundles[bodyBundleIndexB].LinearVelocity.Y, innerIndexB),
                        GatherScatter.Get(ref simulation.Bodies.VelocityBundles[bodyBundleIndexB].LinearVelocity.Z, innerIndexB));
                    var relativeVelocity = (velocityA - velocityB);
                    var surfaceBasis = new Quaternion(
                        GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].SurfaceBasis.X, innerIndex),
                        GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].SurfaceBasis.Y, innerIndex),
                        GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].SurfaceBasis.Z, innerIndex),
                        GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].SurfaceBasis.W, innerIndex));
                    Matrix3x3.CreateFromQuaternion(ref surfaceBasis, out var surfaceBasisMatrix);
                    var normal = surfaceBasisMatrix.Y;
                    var penetrationChange = -dt * Vector3.Dot(relativeVelocity, normal);
                    ref var penetrationDepth = ref GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth0, innerIndex);
                    penetrationDepth += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth1, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth2, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].PenetrationDepth3, innerIndex) += penetrationChange;

                    if (i == 0)
                        //if (penetrationDepth > 0.2)
                        Console.WriteLine($"manifold[{i}] penetration: {penetrationDepth}, velocityA: {velocityA}, velocityB: {velocityB}");

                }
                //for (int i = 0; i < bodyHandles.Length; ++i)
                //{
                //    simulation.Bodies.GetVelocity(bodyHandles[i], out var velocities);
                //    if (velocities.Linear.LengthSquared() > 2)
                //    {
                //        Console.WriteLine($"Body handle {bodyHandles[i]} moving too fast: {velocities.Linear.Y}");
                //    }
                //}


                //Apply some gravity so we can simulate sorta-kinda stacking.
                var bodyBundleCount = simulation.Bodies.BodyCount >> BundleIndexing.VectorShift;
                var impulse = new Vector<float>(-10 * dt);
                for (int i = 0; i < bodyBundleCount; ++i)
                {
                    //(We're using an impulse rather than direct velocity change just because we're being lazy about the kinematics.)
                    simulation.Bodies.VelocityBundles[i].LinearVelocity.Y += simulation.Bodies.LocalInertiaBundles[i].InverseMass * impulse;
                }
                //CacheBlaster.Blast();
                //GC.Collect(3, GCCollectionMode.Forced, true);
                timer.Start();
                //simulation.Solver.Update(dt, inverseDt);
                //solveTime += simulation.Solver.ManualNaiveMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.IntermediateMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.NaiveMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.MultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                //simulation.Solver.ContiguousClaimMultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                simulation.Solver.MultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                timer.Stop();
                var energyAfter = simulation.Bodies.GetBodyEnergyHeuristic();
                //var velocityChange = solver.GetVelocityChangeHeuristic();
                //Console.WriteLine($"Constraint velocity change after frame {frameIndex}: {velocityChange}");
                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");
            }

            Console.WriteLine($"Time (ms): {(1e3 * timer.Elapsed.TotalSeconds)}");
            Console.WriteLine($"Solve time (ms): {1e3 * solveTime}");


            simulation.BufferPool.Clear();

        }



    }
}

