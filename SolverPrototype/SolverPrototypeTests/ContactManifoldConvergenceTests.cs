using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;

namespace SolverPrototypeTests
{
    static class ContactManifoldConvergenceTests
    {
        public static void Test()
        {
            //const int bodyCount = 8;
            //SimulationSetup.BuildStackOfBodiesOnGround(bodyCount, false, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);

            SimulationSetup.BuildLattice(32, 8, 32, true, true, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);


            var bodyOptimizer = new BodyLayoutOptimizer(bodies, graph, solver);
            var constraintOptimizer = new ConstraintLayoutOptimizer(bodies, solver);


            //Attempt cache optimization.
            int bodyOptimizationIterations = bodyHandles.Length * 16;
            //bodyOptimizer.PartialIslandOptimizeDFS(bodyHandles.Length); //prejit
            //bodyOptimizer.DumbIncrementalOptimize(); //prejit
            var timer = Stopwatch.StartNew();
            for (int i = 0; i < bodyOptimizationIterations; ++i)
            {
                //bodyOptimizer.PartialIslandOptimizeDFS(64);
                bodyOptimizer.DumbIncrementalOptimize();
            }
            timer.Stop();
            var optimizationTime = timer.Elapsed.TotalSeconds;
            Console.WriteLine($"Finished {bodyOptimizationIterations} body optimizations, time (ms): {optimizationTime * 1e3}, per iteration (us): {optimizationTime * 1e6 / bodyOptimizationIterations}");

            //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
            int constraintCount = 0;
            for (int i = 0; i < solver.Batches.Count; ++i)
            {
                for (int j = 0; j < solver.Batches[i].TypeBatches.Count; ++j)
                {
                    constraintCount += solver.Batches[i].TypeBatches[j].ConstraintCount;
                }
            }
            const int bundlesPerOptimizationRegion = 256;
            int constraintsPerOptimizationRegion = bundlesPerOptimizationRegion * Vector<int>.Count;
            const int regionsPerConstraintOptimizationIteration = 1;
            int constraintOptimizationIterations = 131072;
            //int constraintOptimizationIterations = Math.Max(16,
            //    (int)(1 * 2 * ((long)constraintCount * constraintCount /
            //    ((double)constraintsPerOptimizationRegion * constraintsPerOptimizationRegion)) / regionsPerConstraintOptimizationIteration));

            constraintOptimizer.Update(2, 1); //prejit
            var constraintsToOptimize = constraintsPerOptimizationRegion * regionsPerConstraintOptimizationIteration * constraintOptimizationIterations;
            timer.Restart();
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                constraintOptimizer.Update(bundlesPerOptimizationRegion, regionsPerConstraintOptimizationIteration);
            }
            timer.Stop();
            Console.WriteLine($"Finished constraint optimizations, time (ms): {timer.Elapsed.TotalMilliseconds}" +
                $", per iteration (us): {timer.Elapsed.TotalSeconds * 1e6 / constraintOptimizationIterations}");


            //By construction, none of the constraints share any bodies, so we can solve it all.
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            const int frameCount = 256;
            solver.IterationCount = iterationCount;


            //prejit
            solver.Update(dt, inverseDt);
            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            timer.Reset();
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = bodies.GetBodyEnergyHeuristic();
                //Update the penetration depths associated with the constraints.
                //This simulates actual position integration and repeated contact detection, allowing the constraints to properly spring.
                for (int i = 0; i < constraintHandles.Length; ++i)
                {
                    solver.GetConstraintReference<ContactManifold4TypeBatch>(constraintHandles[i], out var constraint);
                    
                    BundleIndexing.GetBundleIndices(constraint.IndexInTypeBatch, out var bundleIndex, out var innerIndex);
                    ref var bodyReferences = ref constraint.TypeBatch.BodyReferences[bundleIndex];
                    var velocityA =
                        GatherScatter.Get(
                            ref bodies.VelocityBundles[GatherScatter.Get(ref bodyReferences.BundleIndexA, innerIndex)].LinearVelocity.Y,
                            GatherScatter.Get(ref bodyReferences.InnerIndexA, innerIndex));
                    var velocityB =
                        GatherScatter.Get(
                            ref bodies.VelocityBundles[GatherScatter.Get(ref bodyReferences.BundleIndexB, innerIndex)].LinearVelocity.Y,
                            GatherScatter.Get(ref bodyReferences.InnerIndexB, innerIndex));
                    var penetrationChange = dt * (velocityA - velocityB);
                    ref var penetrationDepth = ref GatherScatter.Get(ref constraint.TypeBatch.PrestepData[bundleIndex].Contact0.PenetrationDepth, innerIndex);
                    penetrationDepth += penetrationChange;
                    GatherScatter.Get(ref constraint.TypeBatch.PrestepData[bundleIndex].Contact1.PenetrationDepth, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref constraint.TypeBatch.PrestepData[bundleIndex].Contact2.PenetrationDepth, innerIndex) += penetrationChange;
                    GatherScatter.Get(ref constraint.TypeBatch.PrestepData[bundleIndex].Contact3.PenetrationDepth, innerIndex) += penetrationChange;

                    if (i == 0)
                        Console.WriteLine($"contact[{i}] penetration: {penetrationDepth}, velocity: {velocityB}");

                }


                //Apply some gravity so we can simulate sorta-kinda stacking.
                var bodyBundleCount = bodies.BodyCount >> BundleIndexing.VectorShift;
                var impulse = new Vector<float>(-10 * dt);
                for (int i = 0; i < bodyBundleCount; ++i)
                {
                    //(We're using an impulse rather than direct velocity change just because we're being lazy about the kinematic.)
                    bodies.VelocityBundles[i].LinearVelocity.Y += bodies.LocalInertiaBundles[i].InverseMass * impulse;
                }
                CacheBlaster.Blast();
                timer.Start();
                solver.Update(dt, inverseDt);
                timer.Stop();
                var energyAfter = bodies.GetBodyEnergyHeuristic();
                //var velocityChange = solver.GetVelocityChangeHeuristic();
                //Console.WriteLine($"Constraint velocity change after frame {frameIndex}: {velocityChange}");
                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");
            }

            Console.WriteLine($"Time (ms): {(1e3 * timer.Elapsed.TotalSeconds)}");

        }


    }
}

