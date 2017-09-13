﻿using BEPUutilities2;
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

namespace SolverPrototypeTests.SpecializedTests
{
    static class MemoryResizeTests
    {
        public static void Test()
        {
            //TODO: Once you have some better constraints to test with (and, say, other systems), this will need to be updated.
            const int width = 8;
            const int height = 8;
            const int length = 8;
            var bodyBuilder = new RegularGridWithKinematicBaseBuilder(new Vector3(1), new Vector3());
            var constraintBuilder = new ContactManifoldConstraintBuilder();
            SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, out var simulation, out var bodyHandles, out var constraintHandles);

            var random = new Random(5);
            for (int i = 0; i < 3000; ++i)
            {
                var sample = random.NextDouble();
                if (sample < 0.05)
                {
                    //Dispose and recreate.
                    simulation.Dispose();
                    simulation.EnsureCapacity(new SimulationAllocationSizes
                    {
                        Bodies = bodyHandles.Length,
                        CollidablesPerType = 128,
                        ShapesPerType = 16,
                        Constraints = constraintHandles.Length,
                        ConstraintCountPerBodyEstimate = 8,
                        ConstraintsPerTypeBatch = 128
                    });
                    SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out bodyHandles, out constraintHandles);
                }
                else if (sample < 0.1)
                {
                    //Clear and recreate.
                    simulation.Clear();
                    SimulationSetup.BuildLattice(bodyBuilder, constraintBuilder, width, height, length, simulation, out bodyHandles, out constraintHandles);
                }
                else
                {
                    //Try to change size.
                    var scale = 4 * random.NextDouble();
                    var sizes = new SimulationAllocationSizes
                    {
                        Bodies = (int)(bodyHandles.Length * scale),
                        CollidablesPerType = (int)(128 * scale),
                        ShapesPerType = (int)(16 * scale),
                        Constraints = (int)(constraintHandles.Length * scale),
                        ConstraintCountPerBodyEstimate = (int)(8 * scale),
                        ConstraintsPerTypeBatch = (int)(128 * scale),
                    };
                    //None of these should ever shrink the size below the current sim size.
                    if (sample < 0.4)
                    {
                        simulation.EnsureCapacity(sizes);
                    }
                    else if (sample < 0.7)
                    {
                        simulation.Compact(sizes);
                    }
                    else
                    {
                        simulation.Resize(sizes);
                    }

                }
            }

            SimulationScrambling.ScrambleBodies(simulation);
            SimulationScrambling.ScrambleConstraints(simulation.Solver);
            SimulationScrambling.ScrambleBodyConstraintLists(simulation);
            SimulationScrambling.AddRemoveChurn<ContactManifold4Constraint>(simulation, 1000, bodyHandles, constraintHandles);

            var threadDispatcher = new SimpleThreadDispatcher(8);

            const int iterations = 10;
            const int internalCompressionIterations = 10;
            for (int i = 0; i < iterations; ++i)
            {
                SimulationScrambling.AddRemoveChurn<ContactManifold4Constraint>(simulation, 10, bodyHandles, constraintHandles);
                GC.Collect(3, GCCollectionMode.Forced, true);
                var start = Stopwatch.GetTimestamp();
                for (int j = 0; j < internalCompressionIterations; ++j)
                {
                    simulation.SolverBatchCompressor.Compress(simulation.BufferPool, threadDispatcher);
                }
            }

            //Attempt cache optimization.
            int bodyOptimizationIterations = bodyHandles.Length * 1;
            simulation.BodyLayoutOptimizer.OptimizationFraction = 0.004f;
            for (int i = 0; i < bodyOptimizationIterations; ++i)
            {
                simulation.BodyLayoutOptimizer.IncrementalOptimize(simulation.BufferPool, threadDispatcher);
            }
            //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
            simulation.ConstraintLayoutOptimizer.OptimizationFraction = 0.044f;
            int constraintOptimizationIterations = 8192;
            for (int i = 0; i < constraintOptimizationIterations; ++i)
            {
                simulation.ConstraintLayoutOptimizer.Update(simulation.BufferPool, threadDispatcher);

            }

            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 8;
            const int frameCount = 256;
            simulation.Solver.IterationCount = iterationCount;

            //If we don't initialize the inertias in a per-frame update, we must do so explicitly.
            simulation.Bodies.LocalInertias.CopyTo(0, ref simulation.Bodies.Inertias, 0, simulation.Bodies.LocalInertias.Length);

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
                        GatherScatter.Get(ref simulation.Bodies.Velocities[bodyBundleIndexA].LinearVelocity.X, innerIndexA),
                        GatherScatter.Get(ref simulation.Bodies.Velocities[bodyBundleIndexA].LinearVelocity.Y, innerIndexA),
                        GatherScatter.Get(ref simulation.Bodies.Velocities[bodyBundleIndexA].LinearVelocity.Z, innerIndexA));
                    var velocityB = new Vector3(
                        GatherScatter.Get(ref simulation.Bodies.Velocities[bodyBundleIndexB].LinearVelocity.X, innerIndexB),
                        GatherScatter.Get(ref simulation.Bodies.Velocities[bodyBundleIndexB].LinearVelocity.Y, innerIndexB),
                        GatherScatter.Get(ref simulation.Bodies.Velocities[bodyBundleIndexB].LinearVelocity.Z, innerIndexB));
                    var relativeVelocity = (velocityA - velocityB);
                     Vector3 normal;
                    unsafe { var mmhmm = &normal; }
                    GatherScatter.GetLane(ref typeBatch.PrestepData[bundleIndex].Normal.X, innerIndex, ref normal.X, 3);
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

                //Apply some gravity so we can simulate sorta-kinda stacking.
                var bodyBundleCount = simulation.Bodies.BodyCount >> BundleIndexing.VectorShift;
                var impulse = new Vector<float>(-10 * dt);
                for (int i = 0; i < bodyBundleCount; ++i)
                {
                    //(We're using an impulse rather than direct velocity change just because we're being lazy about the kinematics.)
                    simulation.Bodies.Velocities[i].LinearVelocity.Y += simulation.Bodies.LocalInertias[i].InverseMass * impulse;
                }
                simulation.Solver.MultithreadedUpdate(threadDispatcher, simulation.BufferPool, dt);
                var energyAfter = simulation.Bodies.GetBodyEnergyHeuristic();
                //var velocityChange = solver.GetVelocityChangeHeuristic();
                //Console.WriteLine($"Constraint velocity change after frame {frameIndex}: {velocityChange}");
                Console.WriteLine($"Body energy {frameIndex}: {energyAfter}, delta: {energyAfter - energyBefore}");

                for (int resizeIndex = 0; resizeIndex < 100; ++resizeIndex)
                {
                    var scale = 4 * random.NextDouble();
                    var sizes = new SimulationAllocationSizes
                    {
                        Bodies = (int)(bodyHandles.Length * scale),
                        CollidablesPerType = (int)(bodyHandles.Length * scale),
                        ShapesPerType = (int)(16 * scale),
                        Constraints = (int)(constraintHandles.Length * scale),
                        ConstraintCountPerBodyEstimate = (int)(8 * scale),
                        ConstraintsPerTypeBatch = (int)(128 * scale)
                    };
                    var sample = random.NextDouble();
                    if (sample < 0.33)
                    {
                        simulation.EnsureCapacity(sizes);
                    }
                    else if (sample < 0.66)
                    {
                        simulation.Compact(sizes);
                    }
                    else
                    {
                        simulation.Resize(sizes);
                    }
                }
            }



            threadDispatcher.Dispose();
            simulation.BufferPool.Clear();

        }



    }
}

