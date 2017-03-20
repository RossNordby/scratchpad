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
        static int CreateManifoldConstraint(int bodyAHandle, int bodyBHandle, Simulation simulation, ref Vector3 right, ref Vector3 up, ref Vector3 forward)
        {
            var description = new ContactManifold4Constraint
            {
                //By convention, normal faces from B to A.
                Normal = new Vector3(0, -1, 0),
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

            var handle = simulation.Add(bodyAHandle, bodyBHandle, ref description);

            simulation.Solver.GetDescription<ContactManifold4Constraint>(handle, out var testDescription);
            return handle;
        }


        public static void ScrambleBodies(Simulation simulation)
        {
            //Having every single body in order is pretty unrealistic. In a real application, churn and general lack of care will result in 
            //scrambled body versus constraint memory access patterns. That's a big increase in cache misses.
            //Scrambling the body array simulates this.
            //Given a sufficiently large added overhead, it would benefit the engine to include runtime cache optimization.
            //That is, move the memory location of bodies (and constraints, within type batches) to maximize the number of accesses to already-cached bodies.

            Random random = new Random(5);
            for (int i = simulation.Bodies.BodyCount - 1; i >= 1; --i)
            {
                //This helper function handles the updates that have to be performed across all body-sensitive systems.
                BodyLayoutOptimizer.SwapBodyLocation(simulation.Bodies, simulation.ConstraintGraph, simulation.Solver, i, random.Next(i));
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

        public static void BuildStackOfBodiesOnGround(int bodyCount,
            out Simulation simulation, out int[] bodyHandles, out int[] constraintHandles)
        {
            simulation = new Simulation(
                initialBodyCapacity: bodyCount,
                initialConstraintCapacity: bodyCount - 1,
                minimumCapacityPerTypeBatch: bodyCount / 2,
                initialConstraintCountPerBodyEstimate: 2);
            bodyHandles = new int[bodyCount];
            //Body 0 is a stationary kinematic acting as the ground.
            {
                var description = new BodyDescription();
                var handleIndex = simulation.Add(ref description);
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
                var handleIndex = simulation.Add(ref description);
                bodyHandles[i] = handleIndex;

            }
            ConstraintTypeIds.Register<ContactManifold4TypeBatch>();


            var right = new Vector3(1, 0, 0);
            var up = new Vector3(0, 1, 0);
            var forward = new Vector3(0, 0, -1);
            constraintHandles = new int[bodyCount - 1];

            for (int bodyIndex = 0; bodyIndex < bodyCount - 1; ++bodyIndex)
            {
                constraintHandles[bodyIndex] = CreateManifoldConstraint(bodyHandles[bodyIndex], bodyHandles[bodyIndex + 1], simulation, ref right, ref up, ref forward);
            }

        }

        public static void BuildLattice(int width, int height, int length, out Simulation simulation,
            out int[] bodyHandles, out int[] constraintHandles)
        {
            var bodyCount = width * height * length;
            simulation = new Simulation(
                initialBodyCapacity: (int)Math.Ceiling(bodyCount / (double)Vector<int>.Count),
                initialConstraintCapacity: bodyCount * 3,
                minimumCapacityPerTypeBatch: (bodyCount * 3) / 6,
                initialConstraintCountPerBodyEstimate: 6);
            bodyHandles = new int[bodyCount];
            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                for (int rowIndex = 0; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        var bodyDescription = new BodyDescription { LocalInertia = new BodyInertia { InverseMass = rowIndex > 0 ? 1 : 0 } };
                        bodyHandles[sliceIndex * (height * width) + rowIndex * width + columnIndex] = simulation.Add(ref bodyDescription);
                    }
                }
            }
            ConstraintTypeIds.Register<ContactManifold4TypeBatch>();


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
                            constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[bodyAIndex - 1], bodyHandles[bodyAIndex], simulation, ref forward, ref right, ref up);
                        }
                        constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[bodyAIndex - width], bodyHandles[bodyAIndex], simulation, ref right, ref up, ref forward);
                        if (sliceIndex > 0)
                        {
                            constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[bodyAIndex - height * width], bodyHandles[bodyAIndex], simulation, ref right, ref forward, ref up);
                        }


                    }
                }
            }
            //Secretly, the constraint handles are just handle[i] = i, and storing them is a little silly.
            Array.Resize(ref constraintHandles, constraintIndex);


        }


        struct RemovedConstraint
        {
            public ContactManifold4Constraint Description;
            public int BodyA;
            public int BodyB;
        }

        struct BodyEntry
        {
            public int Handle;
            public BodyDescription Description;
        }

        struct BodyIndexEnumerator : IForEach<int>
        {
            public Bodies Bodies;
            public int[] HandleToEntryIndex;
            public int EntryIndexA;
            public int EntryIndexB;
            public int IndexInConstraint;

            public BodyIndexEnumerator(Bodies bodies, int[] handleToEntryIndex)
            {
                Bodies = bodies;
                HandleToEntryIndex = handleToEntryIndex;
                EntryIndexA = EntryIndexB = IndexInConstraint = 0;

            }
            public void LoopBody(int connectedBodyIndex)
            {
                var entryIndex = HandleToEntryIndex[Bodies.IndexToHandle[connectedBodyIndex]];
                if (IndexInConstraint == 0)
                    EntryIndexA = entryIndex;
                else
                    EntryIndexB = entryIndex;
                ++IndexInConstraint;
            }
        }

        struct ConstraintRemover : IForEach<ConstraintConnectivityGraph.BodyConstraintReference>
        {
            Simulation simulation;
            int[] handleToEntryIndex;
            List<RemovedConstraint> removedConstraints;
            public ConstraintRemover(Simulation simulation, int[] handleToEntryIndex, List<RemovedConstraint> removedConstraints)
            {
                this.simulation = simulation;
                this.handleToEntryIndex = handleToEntryIndex;
                this.removedConstraints = removedConstraints;

            }
            public void LoopBody(ConstraintConnectivityGraph.BodyConstraintReference constraint)
            {
                RemoveConstraint(simulation, constraint.ConnectingConstraintHandle, handleToEntryIndex, removedConstraints);
            }
        }
        struct ConstraintChecker : IForEach<ConstraintConnectivityGraph.BodyConstraintReference>
        {
            Simulation simulation;
            int[] handleToEntryIndex;
            List<RemovedConstraint> removedConstraints;
            public ConstraintChecker(Simulation simulation, int[] handleToEntryIndex, List<RemovedConstraint> removedConstraints)
            {
                this.simulation = simulation;
                this.handleToEntryIndex = handleToEntryIndex;
                this.removedConstraints = removedConstraints;

            }
            public void LoopBody(ConstraintConnectivityGraph.BodyConstraintReference constraint)
            {
                Console.WriteLine($"Existing constraint: {constraint.ConnectingConstraintHandle}");
            }
        }

        static void RemoveConstraint(Simulation simulation, int constraintHandle, int[] handleToEntry, List<RemovedConstraint> removedConstraints)
        {
            //Enumerate over the body indices referenced by the constraint and convert them to out of engine identities so that they don't get lost.
            simulation.Solver.GetConstraintReference(constraintHandle, out var reference);

            var bodyIndexEnumerator = new BodyIndexEnumerator(simulation.Bodies, handleToEntry);
            reference.TypeBatch.EnumerateConnectedBodyIndices(reference.IndexInTypeBatch, ref bodyIndexEnumerator);
            simulation.Solver.GetDescription(constraintHandle, out ContactManifold4Constraint description);
            simulation.RemoveConstraint(constraintHandle);
            removedConstraints.Add(new RemovedConstraint { Description = description, BodyA = bodyIndexEnumerator.EntryIndexA, BodyB = bodyIndexEnumerator.EntryIndexB });
        }

        struct ConstraintBodyValidationEnumerator : IForEach<int>
        {
            public Simulation Simulation;
            public int ConstraintHandle;
            public void LoopBody(int bodyIndex)
            {
                //The body in this constraint should both:
                //1) have a handle associated with it, and 
                //2) the constraint graph list for the body should include the constraint handle.
                Debug.Assert(Simulation.Bodies.IndexToHandle[bodyIndex] >= 0);
                Debug.Assert(Simulation.ConstraintGraph.BodyIsConstrainedBy(bodyIndex, ConstraintHandle));
            }
        }

        [Conditional("DEBUG")]
        static void ValidateConstraints(Simulation simulation)
        {
            for (int batchIndex = 0; batchIndex < simulation.Solver.Batches.Count; ++batchIndex)
            {
                var batch = simulation.Solver.Batches[batchIndex];
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.ConstraintCount; ++indexInTypeBatch)
                    {
                        var constraintHandle = typeBatch.IndexToHandle[indexInTypeBatch];
                        var constraintLocation = simulation.Solver.HandlesToConstraints[constraintHandle];
                        Debug.Assert(
                            constraintLocation.IndexInTypeBatch == indexInTypeBatch &&
                            batch.TypeIndexToTypeBatchIndex[constraintLocation.TypeId] == typeBatchIndex &&
                            constraintLocation.BatchIndex == batchIndex, "The constraint location stored by the solver should agree with the actual type batch entries.");
                        ConstraintBodyValidationEnumerator enumerator;
                        enumerator.ConstraintHandle = constraintHandle;
                        enumerator.Simulation = simulation;
                        typeBatch.EnumerateConnectedBodyIndices(indexInTypeBatch, ref enumerator);
                    }
                }
            }
        }

        public static void AddRemoveChurn(Simulation simulation, int iterations)
        {
            //Take a snapshot of the body descriptions.
            var bodyEntries = new BodyEntry[simulation.Bodies.BodyCount];
            var handleToEntry = new int[simulation.Bodies.HandleToIndex.Length];
            for (int i = 0; i < handleToEntry.Length; ++i)
                handleToEntry[i] = -1;
            for (int i = 0; i < simulation.Bodies.BodyCount; ++i)
            {
                ref var body = ref bodyEntries[i];
                body.Handle = simulation.Bodies.IndexToHandle[i];
                simulation.Bodies.GetLocalInertia(body.Handle, out body.Description.LocalInertia);
                simulation.Bodies.GetVelocity(body.Handle, out body.Description.Velocity);
                handleToEntry[body.Handle] = i;
            }
            int originalConstraintCount = 0;
            foreach (var batch in simulation.Solver.Batches)
            {
                foreach (var typeBatch in batch.TypeBatches)
                {
                    originalConstraintCount += typeBatch.ConstraintCount;
                }
            }

            //Any time a body is removed, the handle in the associated body entry must be updated to -1.
            //All constraints refer to bodies by their out-of-engine identity so that everything stays robust in the face of adds and removes.
            var removedConstraints = new List<RemovedConstraint>();
            var removedBodies = new List<int>();
            var random = new Random(5);

            var constraintActionProbability = originalConstraintCount > 0 ? 1 - (double)simulation.Bodies.BodyCount / originalConstraintCount : 0;

            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                if (random.NextDouble() < constraintActionProbability)
                {
                    //Constraint action.
                    var constraintRemovalProbability = (originalConstraintCount - removedConstraints.Count) / (double)originalConstraintCount;
                    if (random.NextDouble() < constraintRemovalProbability)
                    {
                        //Remove a constraint.
                        var batch = simulation.Solver.Batches[random.Next(simulation.Solver.Batches.Count)];
                        Debug.Assert(batch.TypeBatches.Count > 0, "Any batch that exists should have a type batch in it, otherwise why is the batch still around?");
                        var typeBatch = batch.TypeBatches[random.Next(batch.TypeBatches.Count)];
                        var indexInTypeBatch = random.Next(typeBatch.ConstraintCount);
                        var constraintHandle = typeBatch.IndexToHandle[indexInTypeBatch];

                        RemoveConstraint(simulation, constraintHandle, handleToEntry, removedConstraints);
                        Console.WriteLine($"Removed constraint, former handle: {constraintHandle}");
                        ValidateConstraints(simulation);
                    }
                    else if (removedConstraints.Count > 0)
                    {
                        //Add a constraint.
                        int attemptCount = 0;
                        do
                        {
                            //There's no guarantee that the bodies involved with the removed constraint are actually in the simulation.
                            //Rather than doing anything clever, just retry a few times.
                            var constraintIndex = random.Next(removedConstraints.Count);
                            var constraint = removedConstraints[constraintIndex];
                            int handleA, handleB;
                            if ((handleA = bodyEntries[constraint.BodyA].Handle) >= 0 && (handleB = bodyEntries[constraint.BodyB].Handle) >= 0)
                            {
                                //The constraint is addable.
                                var constraintHandle = simulation.Add(handleA, handleB, ref constraint.Description);
                                Console.WriteLine($"Added constraint, handle: {constraintHandle}");
                                break;
                            }
                        } while (attemptCount < 10);

                        ValidateConstraints(simulation);
                    }
                }
                else
                {
                    //Body action.
                    var bodyRemovalProbability = (bodyEntries.Length - removedBodies.Count) / (double)bodyEntries.Length;
                    if (random.NextDouble() < bodyRemovalProbability)
                    {
                        //Remove a body.
                        var removedBodyIndex = random.Next(simulation.Bodies.BodyCount);
                        //All constraints associated with the body have to be removed first.
                        var constraintRemover = new ConstraintRemover(simulation, handleToEntry, removedConstraints);
                        simulation.ConstraintGraph.EnumerateConstraints(removedBodyIndex, ref constraintRemover);
                        var constraintChecker = new ConstraintChecker(simulation, handleToEntry, removedConstraints);
                        simulation.ConstraintGraph.EnumerateConstraints(removedBodyIndex, ref constraintChecker);
                        var handle = simulation.Bodies.IndexToHandle[removedBodyIndex];
                        simulation.RemoveBody(handle);
                        bodyEntries[handleToEntry[handle]].Handle = -1;
                        removedBodies.Add(handleToEntry[handle]);
                        handleToEntry[handle] = -1;
                        Console.WriteLine($"Removed body, former handle: {handle}");
                        ValidateConstraints(simulation);
                    }
                    else if (removedBodies.Count > 0)
                    {
                        //Add a body.
                        var toAddIndex = random.Next(removedBodies.Count);
                        var toAdd = removedBodies[toAddIndex];
                        removedBodies.RemoveAt(toAddIndex);
                        var bodyHandle = simulation.Add(ref bodyEntries[toAdd].Description);
                        handleToEntry[bodyHandle] = toAdd;
                        bodyEntries[toAdd].Handle = bodyHandle;
                        Console.WriteLine($"Added body, handle: {bodyHandle}");
                        ValidateConstraints(simulation);
                    }
                }
            }

            //Go ahead and add everything back so the outer test can proceed unaffected. Theoretically.
            for (int i = 0; i < removedBodies.Count; ++i)
            {
                var bodyHandle = simulation.Add(ref bodyEntries[removedBodies[i]].Description);
                bodyEntries[removedBodies[i]].Handle = bodyHandle;
                handleToEntry[bodyHandle] = removedBodies[i];
            }

            for (int i = 0; i < removedConstraints.Count; ++i)
            {
                var constraint = removedConstraints[i];
                simulation.Add(bodyEntries[constraint.BodyA].Handle, bodyEntries[constraint.BodyB].Handle, ref constraint.Description);
            }
            var newConstraintCount = 0;
            foreach (var batch in simulation.Solver.Batches)
            {
                foreach (var typeBatch in batch.TypeBatches)
                {
                    newConstraintCount += typeBatch.ConstraintCount;
                }
            }
            Debug.Assert(newConstraintCount == originalConstraintCount);
            Debug.Assert(bodyEntries.Length == simulation.Bodies.BodyCount);

        }
    }
}

