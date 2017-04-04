using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;

namespace SolverPrototypeTests
{
    static class ContactManifoldConvergenceTests
    {
        public static void Test()
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);

            SimulationSetup.BuildLattice(32, 32, 32, out var simulation, out var bodyHandles, out var constraintHandles);

            //SimulationSetup.ScrambleBodies(simulation);
            //SimulationSetup.ScrambleConstraints(simulation.Solver);

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
            int bodyOptimizationIterations = bodyHandles.Length * 16;
            //bodyOptimizer.PartialIslandOptimizeDFS(bodyHandles.Length); //prejit
            //simulation.BodyLayoutOptimizer.DumbIncrementalOptimize(); //prejit
            var timer = Stopwatch.StartNew();
            for (int i = 0; i < bodyOptimizationIterations; ++i)
            {
                //bodyOptimizer.PartialIslandOptimizeDFS(64);
                //simulation.BodyLayoutOptimizer.DumbIncrementalOptimize();
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
            const int regionsPerConstraintOptimizationIteration = 1;
            //int constraintOptimizationIterations = 131072;
            int constraintOptimizationIterations = Math.Max(16,
                (int)(1 * 2 * ((long)constraintCount * constraintCount /
                ((double)constraintsPerOptimizationRegion * constraintsPerOptimizationRegion)) / regionsPerConstraintOptimizationIteration));

            //simulation.ConstraintLayoutOptimizer.Update(2, 1, simulation.BufferPool); //prejit
            var constraintsToOptimize = constraintsPerOptimizationRegion * regionsPerConstraintOptimizationIteration * constraintOptimizationIterations;
            timer.Restart();
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                //simulation.ConstraintLayoutOptimizer.Update(bundlesPerOptimizationRegion, regionsPerConstraintOptimizationIteration, simulation.BufferPool);
            }
            timer.Stop();
            Console.WriteLine($"Finished constraint optimizations, time (ms): {timer.Elapsed.TotalMilliseconds}" +
                $", per iteration (us): {timer.Elapsed.TotalSeconds * 1e6 / constraintOptimizationIterations}");


            //By construction, none of the constraints share any bodies, so we can solve it all.
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            const int frameCount = 128;
            simulation.Solver.IterationCount = iterationCount;


            //prejit
            simulation.Solver.Update(dt, inverseDt);
            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            timer.Reset();
            var threadPool = new TPLPool();
            //var threadPool = new NotQuiteAThreadPool();
            Console.WriteLine($"Using {threadPool.ThreadCount} workers.");
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
                    ref var bodyReferences = ref typeBatch.BodyReferences[bundleIndex];
                    var velocityA =
                        GatherScatter.Get(
                            ref simulation.Bodies.VelocityBundles[GatherScatter.Get(ref bodyReferences.BundleIndexA, innerIndex)].LinearVelocity.Y,
                            GatherScatter.Get(ref bodyReferences.InnerIndexA, innerIndex));
                    var velocityB =
                        GatherScatter.Get(
                            ref simulation.Bodies.VelocityBundles[GatherScatter.Get(ref bodyReferences.BundleIndexB, innerIndex)].LinearVelocity.Y,
                            GatherScatter.Get(ref bodyReferences.InnerIndexB, innerIndex));
                    var penetrationChange = dt * (velocityA - velocityB);
                    ref var penetrationDepth = ref GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].Contact0.PenetrationDepth, innerIndex);
                    penetrationDepth += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].Contact1.PenetrationDepth, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].Contact2.PenetrationDepth, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref typeBatch.PrestepData[bundleIndex].Contact3.PenetrationDepth, innerIndex) += penetrationChange;

                    //if (i == 0)
                    //    Console.WriteLine($"contact[{i}] penetration: {penetrationDepth}, velocity: {velocityB}");

                }


                //Apply some gravity so we can simulate sorta-kinda stacking.
                var bodyBundleCount = simulation.Bodies.BodyCount >> BundleIndexing.VectorShift;
                var impulse = new Vector<float>(-10 * dt);
                for (int i = 0; i < bodyBundleCount; ++i)
                {
                    //(We're using an impulse rather than direct velocity change just because we're being lazy about the kinematic.)
                    simulation.Bodies.VelocityBundles[i].LinearVelocity.Y += simulation.Bodies.LocalInertiaBundles[i].InverseMass * impulse;
                }
                CacheBlaster.Blast();
                timer.Start();
                //simulation.Solver.Update(dt, inverseDt);
                simulation.Solver.MultithreadedUpdate(threadPool, simulation.BufferPool, dt, inverseDt);
                timer.Stop();
                var energyAfter = simulation.Bodies.GetBodyEnergyHeuristic();
                //var velocityChange = solver.GetVelocityChangeHeuristic();
                //Console.WriteLine($"Constraint velocity change after frame {frameIndex}: {velocityChange}");
                //Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");
            }

            Console.WriteLine($"Time (ms): {(1e3 * timer.Elapsed.TotalSeconds)}");


            simulation.BufferPool.Clear();
        }


    }
}

