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
            var bodyAIndex = bodies.BodyHandles[bodyAHandle];
            var bodyBIndex = bodies.BodyHandles[bodyBHandle];               

            var description = new ContactManifold4Constraint
            {
                Normal = new Vector3(0, 1, 0),
                SpringSettings = new SpringSettingsAOS
                {
                    NaturalFrequency = (float)(Math.PI * 2 * 60),
                    DampingRatio = 100f,
                    MaximumRecoveryVelocity = 1f
                },
                FrictionCoefficient = 1,
                TangentX = new Vector3(1, 0, 0),
                TangentY = new Vector3(0, 0, 1),
            };

            for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
            {
                ref var contact = ref Unsafe.Add(ref description.Contact0, contactIndex);

                var x = (contactIndex & 1) - 0.5f;
                var z = ((contactIndex & 2) >> 1) - 0.5f;
                var localOffsetA = new Vector3(x, 0.5f, z);
                var localOffsetB = new Vector3(x, -0.5f, z);
                var worldOffsetA = localOffsetA.X * right + localOffsetA.Y * up + localOffsetA.Z * forward;
                var worldOffsetB = localOffsetB.X * right + localOffsetB.Y * up + localOffsetB.Z * forward;
                contact.OffsetA = worldOffsetA;
                contact.OffsetB = worldOffsetB;
                contact.PenetrationDepth = 0.00f;
            }
            
            //TODO: c'mon roslyn you could infer those type parameters if you really put the effort in!
            //Would be nice to figure out a way around this. The best solution might actually look like improving roslyn's inference...
            //There probably exists some unsafe-cast-based solution too, but it would be nice to have some degree of compile time safety!
            solver.Add<ContactManifold4Constraint, ContactManifold4TypeBatch>(bodyAIndex, bodyBIndex, ref description, out var reference, out var handle);
            graph.AddConstraint(bodyAIndex, handle, 0);
            graph.AddConstraint(bodyBIndex, handle, 1);
            return handle;
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

