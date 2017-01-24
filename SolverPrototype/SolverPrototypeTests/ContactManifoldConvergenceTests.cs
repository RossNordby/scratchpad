using BEPUutilities2;
using BEPUutilities2.Collections;
using BEPUutilities2.ResourceManagement;
using SolverPrototype;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    static class ContactManifoldConvergenceTests
    {
        public static void Test()
        {
            const int bodyCount = 32768;
            var bodies = BodyStackBuilder.BuildStackOfBodiesOnGround(bodyCount, true, out var handleIndices);

            ConstraintTypeIds.Register<ContactManifold4TypeBatch>();
            var solver = new Solver(bodies);

            int constraintCount = bodyCount - 1;
            int constraintBundleCount = (int)Math.Ceiling(constraintCount / (double)Vector<float>.Count);
            int[] constraintHandles = new int[constraintCount];

            for (int i = 0; i < constraintCount; ++i)
            {
                var bodyAIndex = bodies.BodyHandles[handleIndices[i]];
                var bodyBIndex = bodies.BodyHandles[handleIndices[i + 1]];

                solver.Allocate<ContactManifold4TypeBatch>(bodyAIndex, bodyBIndex, out var constraintReference, out constraintHandles[i]);


                BundleIndexing.GetBundleIndices(bodyAIndex, out var bodyABundleIndex, out var bodyAInnerIndex);
                BundleIndexing.GetBundleIndices(bodyBIndex, out var bodyBBundleIndex, out var bodyBInnerIndex);
                BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
                ref var constraintBodies = ref constraintReference.TypeBatch.BodyReferences[constraintBundleIndex];
                GatherScatter.Get(ref constraintBodies.BundleIndexA, constraintInnerIndex) = bodyABundleIndex;
                GatherScatter.Get(ref constraintBodies.InnerIndexA, constraintInnerIndex) = bodyAInnerIndex;
                GatherScatter.Get(ref constraintBodies.BundleIndexB, constraintInnerIndex) = bodyBBundleIndex;
                GatherScatter.Get(ref constraintBodies.InnerIndexB, constraintInnerIndex) = bodyBInnerIndex;
                var constraintBundleBaseIndex = constraintBundleIndex << BundleIndexing.VectorShift;
                ++constraintBodies.Count;

                ref var prestep = ref constraintReference.TypeBatch.PrestepData[constraintBundleIndex];

                GatherScatter.Get(ref prestep.SpringSettings.NaturalFrequency, constraintInnerIndex) = (float)(Math.PI * 2 * 60);
                GatherScatter.Get(ref prestep.SpringSettings.DampingRatio, constraintInnerIndex) = 100f;
                GatherScatter.Get(ref prestep.SpringSettings.MaximumRecoveryVelocity, constraintInnerIndex) = 1f;
                //Normal goes from B to A by convention.
                GatherScatter.Get(ref prestep.Normal.Y, constraintInnerIndex) = -1;

                for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
                {
                    //TODO: Normal and spring settings should really be shared on convex manifolds.
                    ref var contact = ref Unsafe.Add(ref prestep.Contact0, contactIndex);

                    var x = (contactIndex & 1) - 0.5f;
                    var z = ((contactIndex & 2) >> 1) - 0.5f;
                    GatherScatter.Get(ref contact.OffsetA.X, constraintInnerIndex) = x;
                    GatherScatter.Get(ref contact.OffsetA.Y, constraintInnerIndex) = 0.5f;
                    GatherScatter.Get(ref contact.OffsetA.Z, constraintInnerIndex) = z;
                    GatherScatter.Get(ref contact.OffsetB.X, constraintInnerIndex) = x;
                    GatherScatter.Get(ref contact.OffsetB.Y, constraintInnerIndex) = -0.5f;
                    GatherScatter.Get(ref contact.OffsetB.Z, constraintInnerIndex) = z;
                    GatherScatter.Get(ref contact.PenetrationDepth, constraintInnerIndex) = 0.00f;
                }

                GatherScatter.Get(ref prestep.FrictionCoefficient, constraintInnerIndex) = 1;
                GatherScatter.Get(ref prestep.TangentX.X, constraintInnerIndex) = 1;
                GatherScatter.Get(ref prestep.TangentY.Z, constraintInnerIndex) = 1;


            }

            //By construction, none of the constraints share any bodies, so we can solve it all.
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 1;
            const int frameCount = 256;
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
                for (int i = 0; i < constraintCount; ++i)
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

