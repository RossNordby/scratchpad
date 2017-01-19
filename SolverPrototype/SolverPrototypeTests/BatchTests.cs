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
    static class BatchTests
    {
        public static void Test()
        {
            Bodies bodies = new Bodies();
            const int bodyCount = 1024;
            var handleIndices = new int[bodyCount];
            //Body 0 is a stationary kinematic acting as the ground.
            {
                var description = new BodyDescription();
                var handleIndex = bodies.Add(ref description);
                handleIndices[0] = handleIndex;
            }

            //The remaining bodies form an extremely tall stack with initial velocities set as if this is the first frame of impact.
            for (int i = 1; i < bodyCount; ++i)
            {
                var description = new BodyDescription
                {
                    LocalInertia = new BodyInertia
                    {
                        InverseInertiaTensor = new Matrix3x3
                        {
                            X = new Vector3(1, 0, 0),
                            Y = new Vector3(0, 1, 0),
                            Z = new Vector3(0, 0, 1),
                        },
                        InverseMass = 1
                    },
                    Velocity = new BodyVelocity { Linear = new Vector3(0, -1, 0) }
                };
                var handleIndex = bodies.Add(ref description);
                handleIndices[i] = handleIndex;
                
            }

            ConstraintTypeIds.Register<ContactPenetrationTypeBatch>();
            var solver = new Solver(bodies);

            int constraintCount = bodyCount - 1;
            int constraintBundleCount = (int)Math.Ceiling(constraintCount / (double)Vector<float>.Count);

            var accumulatedImpulses = new Vector<float>[constraintBundleCount];
            for (int i = 0; i < constraintCount; ++i)
            {
                var bodyAIndex = bodies.BodyHandles[handleIndices[i]];
                var bodyBIndex = bodies.BodyHandles[handleIndices[i + 1]];

                solver.Allocate<ContactPenetrationTypeBatch>(bodyAIndex, bodyBIndex, out var constraintReference, out var handleIndex);


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
                GatherScatter.Get(ref prestep.SpringSettings.DampingRatio, constraintInnerIndex) = 1;
                GatherScatter.Get(ref prestep.SpringSettings.MaximumRecoveryVelocity, constraintInnerIndex) = 1;

                //Normal goes from B to A by convention.
                GatherScatter.Get(ref prestep.Normal.Y, constraintInnerIndex) = -1;
                GatherScatter.Get(ref prestep.OffsetA.Y, constraintInnerIndex) = 0.5f;
                GatherScatter.Get(ref prestep.OffsetB.Y, constraintInnerIndex) = -0.5f;
                GatherScatter.Get(ref prestep.PenetrationDepth, constraintInnerIndex) = 0.00f;


            }

            //By construction, none of the constraints share any bodies, so we can solve it all.
            const float inverseDt = 60f;
            const float dt = 1 / inverseDt;
            const int iterationCount = 5;
            const int frameCount = 256;
            solver.IterationCount = iterationCount;


            //prejit
            //solver.Update(dt, inverseDt);
            //Technically we're not doing any position integration or collision detection yet, so these frames are pretty meaningless.
            var start = Stopwatch.GetTimestamp();
            for (int frameIndex = 0; frameIndex < frameCount; ++frameIndex)
            {
                var energyBefore = bodies.GetBodyEnergyHeuristic();
                solver.Update(dt, inverseDt);
                var energyAfter = bodies.GetBodyEnergyHeuristic();
                //var velocityChange = solver.GetVelocityChangeHeuristic();
                //Console.WriteLine($"Constraint velocity change after frame {frameIndex}: {velocityChange}");
                Console.WriteLine($"Body energy change {frameIndex}: {energyAfter - energyBefore}");
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Time (ms): {(1e3 * (end - start)) / Stopwatch.Frequency}");

        }


    }
}

