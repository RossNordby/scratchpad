using BEPUutilities2;
using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototypeTests
{
    static class SimulationSetup
    {
        static int CreateManifoldConstraint(int bodyAHandle, int bodyBHandle, Bodies bodies, Solver solver, ConstraintConnectivityGraph graph, ref Vector3 right, ref Vector3 up, ref Vector3 forward)
        {
            solver.Allocate<ContactManifold4TypeBatch>(bodyAHandle, bodyBHandle, out var constraintReference, out var constraintHandle);
            var bodyAIndex = bodies.BodyHandles[bodyAHandle];
            var bodyBIndex = bodies.BodyHandles[bodyBHandle];
            graph.AddConstraint(bodyAIndex, constraintHandle, 0);
            graph.AddConstraint(bodyBIndex, constraintHandle, 1);


            BundleIndexing.GetBundleIndices(bodyAIndex, out var bodyABundleIndex, out var bodyAInnerIndex);
            BundleIndexing.GetBundleIndices(bodyBIndex, out var bodyBBundleIndex, out var bodyBInnerIndex);
            BundleIndexing.GetBundleIndices(constraintReference.IndexInTypeBatch, out var constraintBundleIndex, out var constraintInnerIndex);
            ref var constraintBodies = ref constraintReference.TypeBatch.BodyReferences[constraintBundleIndex];
            Debug.Assert(constraintBodies.Count < Vector<int>.Count);
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
                ref var contact = ref Unsafe.Add(ref prestep.Contact0, contactIndex);

                var x = (contactIndex & 1) - 0.5f;
                var z = ((contactIndex & 2) >> 1) - 0.5f;
                var localOffsetA = new Vector3(x, 0.5f, z);
                var localOffsetB = new Vector3(x, -0.5f, z);
                var worldOffsetA = localOffsetA.X * right + localOffsetA.Y * up + localOffsetA.Z * forward;
                var worldOffsetB = localOffsetB.X * right + localOffsetB.Y * up + localOffsetB.Z * forward;
                GatherScatter.Get(ref contact.OffsetA.X, constraintInnerIndex) = worldOffsetA.X;
                GatherScatter.Get(ref contact.OffsetA.Y, constraintInnerIndex) = worldOffsetA.Y;
                GatherScatter.Get(ref contact.OffsetA.Z, constraintInnerIndex) = worldOffsetA.Z;
                GatherScatter.Get(ref contact.OffsetB.X, constraintInnerIndex) = worldOffsetB.X;
                GatherScatter.Get(ref contact.OffsetB.Y, constraintInnerIndex) = worldOffsetB.Y;
                GatherScatter.Get(ref contact.OffsetB.Z, constraintInnerIndex) = worldOffsetB.Z;
                GatherScatter.Get(ref contact.PenetrationDepth, constraintInnerIndex) = 0.00f;
            }

            GatherScatter.Get(ref prestep.FrictionCoefficient, constraintInnerIndex) = 1;
            GatherScatter.Get(ref prestep.TangentX.X, constraintInnerIndex) = 1;
            GatherScatter.Get(ref prestep.TangentY.Z, constraintInnerIndex) = 1;

            solver.Add(bodyAIndex, bodyBIndex, ref description, out var reference, out var handle);
            return constraintHandle;
        }


        public static void ScrambleBodies(Bodies bodies)
        {
            //Having every single body in order is pretty unrealistic. In a real application, churn and general lack of care will result in 
            //scrambled body versus constraint memory access patterns. That's a big increase in cache misses.
            //Scrambling the body array simulates this.
            //Given a sufficiently large added overhead, it would benefit the engine to include runtime cache optimization.
            //That is, move the memory location of bodies (and constraints, within type batches) to maximize the number of accesses to already-cached bodies.

            Random random = new Random(5);
            for (int i = bodies.BodyCount - 1; i >= 0; --i)
            {
                bodies.Swap(i, random.Next(i));
            }

        }

        public static void ScrambleConstraints(Solver solver)
        {
            Random random = new Random(5);
            for (int i = 0; i < solver.Batches.Count; ++i)
            {
                for (int j = 0; j < solver.Batches[i].TypeBatches.Count; ++j)
                {
                    solver.Batches[i].TypeBatches[j].Scramble(random, solver.HandlesToConstraints);
                }
            }
        }

        public static void BuildStackOfBodiesOnGround(int bodyCount, bool scrambleBodies, bool scrambleConstraints,
            out Bodies bodies, out Solver solver, out ConstraintConnectivityGraph graph, out int[] bodyHandles, out int[] constraintHandles)
        {
            bodies = new Bodies();
            bodyHandles = new int[bodyCount];
            //Body 0 is a stationary kinematic acting as the ground.
            {
                var description = new BodyDescription();
                var handleIndex = bodies.Add(ref description);
                bodyHandles[0] = handleIndex;
            }

            //The remaining bodies form an extremely tall stack. 
            for (int i = 1; i < bodyCount; ++i)
            {
                var description = new BodyDescription
                {
                    LocalInertia = new BodyInertia
                    {
                        //InverseInertiaTensor = new Matrix3x3
                        //{
                        //    X = new Vector3(1, 0, 0),
                        //    Y = new Vector3(0, 1, 0),
                        //    Z = new Vector3(0, 0, 1),
                        //},
                        InverseMass = 1
                    },
                };
                var handleIndex = bodies.Add(ref description);
                bodyHandles[i] = handleIndex;

            }
            if (scrambleBodies)
                ScrambleBodies(bodies);
            ConstraintTypeIds.Register<ContactManifold4TypeBatch>();
            solver = new Solver(bodies, initialCapacity: bodyCount * 3);
            graph = new ConstraintConnectivityGraph(solver, bodyCount, 6);

            for (int i = 0; i < bodies.BodyCount; ++i)
            {
                graph.AddBodyList(i);
            }




            var right = new Vector3(1, 0, 0);
            var up = new Vector3(0, 1, 0);
            var forward = new Vector3(0, 0, -1);
            constraintHandles = new int[bodyCount - 1];

            for (int bodyIndex = 0; bodyIndex < bodyCount - 1; ++bodyIndex)
            {
                constraintHandles[bodyIndex] = CreateManifoldConstraint(bodyHandles[bodyIndex], bodyHandles[bodyIndex + 1], bodies, solver, graph, ref right, ref up, ref forward);
            }
            
            if (scrambleConstraints)
            {
                ScrambleConstraints(solver);
            }
        }

        public static void BuildLattice(int width, int height, int length, bool scrambleBodies, bool scrambleConstraints, out Bodies bodies, out Solver solver, out ConstraintConnectivityGraph graph,
            out int[] bodyHandles, out int[] constraintHandles)
        {
            var bodyCount = width * height * length;
            bodies = new Bodies((int)Math.Ceiling(bodyCount / (double)Vector<int>.Count));
            bodyHandles = new int[bodyCount];
            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                for (int rowIndex = 0; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        var bodyDescription = new BodyDescription { LocalInertia = new BodyInertia { InverseMass = rowIndex > 0 ? 1 : 0 } };
                        bodyHandles[sliceIndex * (height * width) + rowIndex * width + columnIndex] = bodies.Add(ref bodyDescription);
                    }
                }
            }
            if (scrambleBodies)
                ScrambleBodies(bodies);
            ConstraintTypeIds.Register<ContactManifold4TypeBatch>();
            int constraintCount = (width - 1) * (height - 1) * (length - 1) +
                width * height;
            solver = new Solver(bodies, initialCapacity: bodyCount * 3);
            graph = new ConstraintConnectivityGraph(solver, bodyCount, 6);

            for (int i = 0; i < bodies.BodyCount; ++i)
            {
                graph.AddBodyList(i);
            }



            var right = new Vector3(1, 0, 0);
            var up = new Vector3(0, 1, 0);
            var forward = new Vector3(0, 0, -1);
            //Note super lazy count initialization. Since we do some wonky stuff with the base we'll just resize later.
            constraintHandles = new int[width * height * length * 3];
            int constraintIndex = 0;
            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                for (int rowIndex = 1; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        var bodyAIndex = sliceIndex * (height * width) + rowIndex * width + columnIndex;
                        //For each lower neighbor, create a connection.
                        //The bottom rows are all kinematic, so don't create connections between them.
                        if (columnIndex > 0)
                        {
                            constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[bodyAIndex - 1], bodyHandles[bodyAIndex], bodies, solver, graph, ref forward, ref right, ref up);
                        }
                        constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[bodyAIndex - width], bodyHandles[bodyAIndex], bodies, solver, graph, ref right, ref up, ref forward);
                        if (sliceIndex > 0)
                        {
                            constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[bodyAIndex - height * width], bodyHandles[bodyAIndex], bodies, solver, graph, ref right, ref forward, ref up);
                        }


                    }
                }
            }
            //Secretly, the constraint handles are just handle[i] = i, and storing them is a little silly.
            Array.Resize(ref constraintHandles, constraintIndex);


            if (scrambleConstraints)
            {
                ScrambleConstraints(solver);
            }
        }
    }
}

