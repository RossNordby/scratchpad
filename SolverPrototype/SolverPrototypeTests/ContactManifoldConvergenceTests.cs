using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using SolverPrototype;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;

namespace SolverPrototypeTests
{
    static class ContactManifoldConvergenceTests
    {
        public static void Test()
        {
            SimulationSetup.BuildStackOfBodiesOnGround(262144, false, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);

            //SimulationSetup.BuildLattice(48, 48, 48, false, out var bodies, out var solver, out var graph, out var bodyHandles, out var constraintHandles);


            var optimizer = new BodyLayoutOptimizer(bodies, graph, solver);



            //Attempt cache optimization.
            int optimizationIterations = bodyHandles.Length * 16;
            //optimizer.PartialIslandOptimizeDFS(bodyHandles.Length); //prejit
            //optimizer.DumbIncrementalOptimize(); //prejit
            var startOptimization = Stopwatch.GetTimestamp();
            for (int i = 0; i < optimizationIterations; ++i)
            {
                //optimizer.PartialIslandOptimizeDFS(64);
                //optimizer.DumbIncrementalOptimize();
            }
            var endOptimization = Stopwatch.GetTimestamp();
            var optimizationTime = (endOptimization - startOptimization) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Finished {optimizationIterations} optimizations, time (ms): {optimizationTime * 1e3}, per iteration (us): {optimizationTime * 1e6 / optimizationIterations}");
            //By construction, none of the constraints share any bodies, so we can solve it all.
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            const int frameCount = 16;
            solver.IterationCount = iterationCount;


            //prejit
            solver.Update(dt, inverseDt);
            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            long totalTicks = 0;
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

                    //if (i == 0)
                    //    Console.WriteLine($"contact[{i}] penetration: {penetrationDepth}, velocity: {velocityB}");

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
                var frameStart = Stopwatch.GetTimestamp();
                solver.Update(dt, inverseDt);
                var frameEnd = Stopwatch.GetTimestamp();
                totalTicks += frameEnd - frameStart;
                var energyAfter = bodies.GetBodyEnergyHeuristic();
                //var velocityChange = solver.GetVelocityChangeHeuristic();
                //Console.WriteLine($"Constraint velocity change after frame {frameIndex}: {velocityChange}");
                //Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");
            }

            Console.WriteLine($"Time (ms): {(1e3 * totalTicks) / Stopwatch.Frequency}");

        }


    }
}

