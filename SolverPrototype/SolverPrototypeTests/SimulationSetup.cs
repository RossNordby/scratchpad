using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

using Quaternion = BEPUutilities2.Quaternion;

namespace SolverPrototypeTests
{
    static class SimulationSetup
    {
        static int CreateManifoldConstraint(int bodyAHandle, int bodyBHandle, Simulation simulation, ref Vector3 unitX, ref Vector3 unitY, ref Vector3 unitZ)
        {
            var description = new ContactManifold4Constraint
            {
                //By convention, normal faces from B to A.
                SpringSettings = new SpringSettingsAOS
                {
                    NaturalFrequency = (float)(Math.PI * 2 * 60),
                    DampingRatio = 100f,
                    MaximumRecoveryVelocity = 1f
                },
                FrictionCoefficient = 1,
            };
            Matrix3x3 basisMatrix;
            basisMatrix.X = unitX;
            basisMatrix.Y = -unitY;
            basisMatrix.Z = -unitZ;
            Debug.Assert(basisMatrix.Determinant() == 1);
            Quaternion.CreateFromRotationMatrix(ref basisMatrix, out description.SurfaceBasis);

            for (int contactIndex = 0; contactIndex < 4; ++contactIndex)
            {
                ref var contact = ref Unsafe.Add(ref description.Contact0, contactIndex);

                var x = (contactIndex & 1) - 0.5f;
                var z = ((contactIndex & 2) >> 1) - 0.5f;
                var localOffsetA = new Vector3(x, 0.5f, z);
                var localOffsetB = new Vector3(x, -0.5f, z);
                var worldOffsetA = localOffsetA.X * unitX + localOffsetA.Y * unitY + localOffsetA.Z * unitZ;
                var worldOffsetB = localOffsetB.X * unitX + localOffsetB.Y * unitY + localOffsetB.Z * unitZ;
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
                    solver.Batches[i].TypeBatches[j].Scramble(random, ref solver.HandleToConstraint);
                }
            }
        }
        public static void ScrambleBodyConstraintLists(Simulation simulation)
        {
            Random random = new Random(5);
            //Body lists are isolated enough that we don't have to worry about a bunch of internal bookkeeping. Just pull the list and mess with it.
            //Note that we cannot change the order of bodies within constraints! That would change behavior.
            for (int bodyIndex = 0; bodyIndex < simulation.Bodies.BodyCount; ++bodyIndex)
            {
                ref var list = ref simulation.ConstraintGraph.GetConstraintList(bodyIndex);
                for (int i = 0; i < list.Count - 1; ++i)
                {
                    ref var currentSlot = ref list[i];
                    ref var otherSlot = ref list[random.Next(i + 1, list.Count)];
                    var currentTemp = currentSlot;
                    currentSlot = otherSlot;
                    otherSlot = currentTemp;
                }
            }
        }

        public static void BuildStackOfBodiesOnGround(int bodyCount,
            out Simulation simulation, out int[] bodyHandles, out int[] constraintHandles)
        {
            simulation = new Simulation(
                new BufferPool(),
                new SimulationAllocationSizes
                {
                    Bodies = bodyCount,
                    Constraints = bodyCount - 1,
                    ConstraintsPerTypeBatch = bodyCount / 2,
                    ConstraintCountPerBodyEstimate = 2
                });
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
                new BufferPool(),
                new SimulationAllocationSizes
                {
                    Bodies = (int)Math.Ceiling(bodyCount / (double)Vector<int>.Count),
                    Constraints = bodyCount * 3,
                    ConstraintsPerTypeBatch = (bodyCount * 3) / 6,
                    ConstraintCountPerBodyEstimate = 6
                });
            bodyHandles = new int[bodyCount];
            int ToId(int columnIndex, int rowIndex, int sliceIndex)
            {
                return sliceIndex * (height * width) + rowIndex * width + columnIndex;
            }
            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                for (int rowIndex = 0; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        var bodyDescription = new BodyDescription { LocalInertia = new BodyInertia { InverseMass = rowIndex > 0 ? 1 : 0 } };
                        var id = ToId(columnIndex, rowIndex, sliceIndex);
                        bodyHandles[id] = simulation.Add(ref bodyDescription);
                    }
                }
            }
            ConstraintTypeIds.Register<ContactManifold4TypeBatch>();


            var unitX = new Vector3(1, 0, 0);
            var unitY = new Vector3(0, 1, 0);
            var unitZ = new Vector3(0, 0, 1);
            //Note super lazy count initialization. Since we do some wonky stuff with the base we'll just resize later.
            constraintHandles = new int[width * height * length * 3];
            int constraintIndex = 0;
            for (int sliceIndex = 0; sliceIndex < length; ++sliceIndex)
            {
                //The bottom rows are all kinematic, so don't create connections between them.
                for (int rowIndex = 1; rowIndex < height; ++rowIndex)
                {
                    for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                    {
                        var bodyAIndex = ToId(columnIndex, rowIndex, sliceIndex);
                        //For each lower neighbor, create a connection.
                        if (columnIndex > 0)
                        {
                            var previousColumnId = ToId(columnIndex - 1, rowIndex, sliceIndex);
                            constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[previousColumnId], bodyHandles[bodyAIndex], simulation, ref unitZ, ref unitX, ref unitY);
                        }
                        var previousRowId = ToId(columnIndex, rowIndex - 1, sliceIndex);
                        constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[previousRowId], bodyHandles[bodyAIndex], simulation, ref unitX, ref unitY, ref unitZ);
                        if (sliceIndex > 0)
                        {
                            var previousSliceId = ToId(columnIndex, rowIndex, sliceIndex - 1);
                            constraintHandles[constraintIndex++] = CreateManifoldConstraint(bodyHandles[previousSliceId], bodyHandles[bodyAIndex], simulation, ref unitY, ref unitZ, ref unitX);
                        }


                    }
                }
            }
            //Secretly, the constraint handles are just handle[i] = i, and storing them is a little silly.
            Array.Resize(ref constraintHandles, constraintIndex);


        }


        struct CachedConstraint
        {
            public ContactManifold4Constraint Description;
            public int BodyA;
            public int BodyB;
        }

        struct BodyEnumerator : IForEach<int>
        {
            public Bodies Bodies;
            public int[] HandlesToIdentity;
            public int IdentityA;
            public int IdentityB;
            public int IndexInConstraint;

            public BodyEnumerator(Bodies bodies, int[] handleToEntryIndex)
            {
                Bodies = bodies;
                HandlesToIdentity = handleToEntryIndex;
                IdentityA = IdentityB = IndexInConstraint = 0;

            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int connectedBodyIndex)
            {
                var entryIndex = HandlesToIdentity[Bodies.IndexToHandle[connectedBodyIndex]];
                if (IndexInConstraint == 0)
                    IdentityA = entryIndex;
                else
                    IdentityB = entryIndex;
                ++IndexInConstraint;
            }
        }


        static void RemoveConstraint(Simulation simulation, int constraintHandle, int[] constraintHandlesToIdentity, int[] constraintHandles, List<int> removedConstraints)
        {
            var constraintIdentity = constraintHandlesToIdentity[constraintHandle];
            constraintHandlesToIdentity[constraintHandle] = -1;
            constraintHandles[constraintIdentity] = -1;
            simulation.RemoveConstraint(constraintHandle);
            removedConstraints.Add(constraintIdentity);
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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void WriteLine(string message)
        {
            //Debug.WriteLine(message);
        }

        [Conditional("DEBUG")]
        static void Validate(Simulation simulation, List<int> removedConstraints, List<int> removedBodies, int originalBodyCount, int originalConstraintCount)
        {
            for (int batchIndex = 0; batchIndex < simulation.Solver.Batches.Count; ++batchIndex)
            {
                var batch = simulation.Solver.Batches[batchIndex];
                if (batchIndex == simulation.Solver.Batches.Count - 1)
                {
                    Debug.Assert(batch.TypeBatches.Count > 0, "While a lower indexed batch may have zero elements (especially while batch compression isn't active), " +
                        "there should never be an empty batch at the end of the list.");
                }
                for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
                {
                    var typeBatch = batch.TypeBatches[typeBatchIndex];
                    Debug.Assert(typeBatch.ConstraintCount > 0, "If a type batch exists, there should be constraints in it.");
                    for (int indexInTypeBatch = 0; indexInTypeBatch < typeBatch.ConstraintCount; ++indexInTypeBatch)
                    {
                        var constraintHandle = typeBatch.IndexToHandle[indexInTypeBatch];
                        var constraintLocation = simulation.Solver.HandleToConstraint[constraintHandle];
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
            var constraintCount = 0;
            foreach (var batch in simulation.Solver.Batches)
            {
                foreach (var typeBatch in batch.TypeBatches)
                {
                    constraintCount += typeBatch.ConstraintCount;
                }
            }

            Debug.Assert(removedConstraints.Count + constraintCount == originalConstraintCount, "Must not have lost (or gained) any constraints!");
            Debug.Assert(removedBodies.Count + simulation.Bodies.BodyCount == originalBodyCount, "Must not have lost (or gained) any bodies!");

        }
        static void FastRemoveAt<T>(List<T> list, int index)
        {
            var lastIndex = list.Count - 1;
            if (lastIndex != index)
            {
                list[index] = list[lastIndex];
            }
            list.RemoveAt(lastIndex);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnAddBody(Simulation simulation, BodyDescription[] bodyDescriptions, int[] bodyHandles, int[] bodyHandlesToIdentity,
            int originalConstraintCount, List<int> removedConstraints, List<int> removedBodies, Random random)
        {
            //Add a body.
            var toAddIndex = random.Next(removedBodies.Count);
            var toAdd = removedBodies[toAddIndex];
            FastRemoveAt(removedBodies, toAddIndex);
            var bodyHandle = simulation.Add(ref bodyDescriptions[toAdd]);
            bodyHandlesToIdentity[bodyHandle] = toAdd;
            bodyHandles[toAdd] = bodyHandle;
            WriteLine($"Added body, handle: {bodyHandle}");
            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, originalConstraintCount);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnRemoveBody(Simulation simulation, int[] bodyHandles, int[] bodyHandlesToIdentity, int[] constraintHandles,
            int[] constraintHandlesToIdentity, CachedConstraint[] constraintDescriptions,
            List<int> removedConstraints, List<int> removedBodies, Random random)
        {
            //Remove a body.
            var removedBodyIndex = random.Next(simulation.Bodies.BodyCount);
            //All constraints associated with the body have to be removed first.
            ref var constraintList = ref simulation.ConstraintGraph.GetConstraintList(removedBodyIndex);
            for (int i = constraintList.Count - 1; i >= 0; --i)
            {
                WriteLine($"Removing constraint (handle: {constraintList[i].ConnectingConstraintHandle}) for a body removal.");
                RemoveConstraint(simulation, constraintList[i].ConnectingConstraintHandle, constraintHandlesToIdentity, constraintHandles, removedConstraints);
            }
#if DEBUG
            Debug.Assert(constraintList.Count == 0, "After we removed all the constraints, the constraint list should be empty! (It's a ref to the actual slot!)");
#endif
            var handle = simulation.Bodies.IndexToHandle[removedBodyIndex];
            simulation.RemoveBody(handle);
            bodyHandles[bodyHandlesToIdentity[handle]] = -1;
            removedBodies.Add(bodyHandlesToIdentity[handle]);
            bodyHandlesToIdentity[handle] = -1;
            WriteLine($"Removed body, former handle: {handle}");
            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, constraintHandles.Length);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnAddConstraint(Simulation simulation, int[] bodyHandles, int[] constraintHandles, int[] constraintHandlesToIdentity,
            CachedConstraint[] constraintDescriptions, List<int> removedConstraints, List<int> removedBodies, Random random)
        {
            //Add a constraint.
            int attemptCount = 0;
            do
            {
                //There's no guarantee that the bodies involved with the removed constraint are actually in the simulation.
                //Rather than doing anything clever, just retry a few times.
                var constraintIdentityIndex = random.Next(removedConstraints.Count);
                var constraintIdentity = removedConstraints[constraintIdentityIndex];
                ref var constraint = ref constraintDescriptions[constraintIdentity];
                int handleA, handleB;
                if ((handleA = bodyHandles[constraint.BodyA]) >= 0 && (handleB = bodyHandles[constraint.BodyB]) >= 0)
                {
                    //The constraint is addable.
                    var constraintHandle = simulation.Add(handleA, handleB, ref constraint.Description);
                    constraintHandles[constraintIdentity] = constraintHandle;
                    constraintHandlesToIdentity[constraintHandle] = constraintIdentity;
                    WriteLine($"Added constraint, handle: {constraintHandle}");
                    FastRemoveAt(removedConstraints, constraintIdentityIndex);
                    break;
                }
            } while (++attemptCount < 10);
            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, constraintHandles.Length);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static void ChurnRemoveConstraint(Simulation simulation, int originalBodyCount,
            int[] constraintHandlesToIdentity, int[] constraintHandles, CachedConstraint[] constraintDescriptions, List<int> removedConstraints, List<int> removedBodies, Random random)
        {
            //Remove a constraint.
            var batchIndex = random.Next(simulation.Solver.Batches.Count);
            var batch = simulation.Solver.Batches[batchIndex];
            Debug.Assert(batchIndex < simulation.Solver.Batches.Count - 1 || batch.TypeBatches.Count > 0,
                "While a lower index batch may end up empty due to a lack of active batch compression, " +
                "the last batch should get removed if it becomes empty since there is no danger of pointer invaldiation.");
            if (batch.TypeBatches.Count > 0)
            {
                var typeBatch = batch.TypeBatches[random.Next(batch.TypeBatches.Count)];
                Debug.Assert(typeBatch.ConstraintCount > 0, "If a type batch exists, it should have constraints in it.");
                var indexInTypeBatch = random.Next(typeBatch.ConstraintCount);
                var constraintHandle = typeBatch.IndexToHandle[indexInTypeBatch];

                RemoveConstraint(simulation, constraintHandle, constraintHandlesToIdentity, constraintHandles, removedConstraints);
                WriteLine($"Removed constraint, former handle: {constraintHandle}");
                Validate(simulation, removedConstraints, removedBodies, originalBodyCount, constraintHandles.Length);
            }
        }

        public static double AddRemoveChurn(Simulation simulation, int iterations, int[] bodyHandles, int[] constraintHandles)
        {
            //There are three levels of 'index' for each object in this test:
            //1) The top level 'identity'. Even when a body or constraint gets readded, the slot in the top level array maintains a pointer to the new handle.
            //2) The in-engine handle. Within the engine, it acts as the identity. The engine only cares about tracking identity between calls to add and remove for any given object.
            //3) The index of the object in memory.
            //As we add and remove stuff, we want to still be able to find a particular constraint by its original identity, so we have to do some work to track that.

            //Take a snapshot of the body descriptions.
            var bodyDescriptions = new BodyDescription[bodyHandles.Length];
            var constraintDescriptions = new CachedConstraint[constraintHandles.Length];
            Debug.Assert(simulation.Bodies.BodyCount == bodyHandles.Length);
            int originalConstraintCount = 0;
            foreach (var batch in simulation.Solver.Batches)
            {
                foreach (var typeBatch in batch.TypeBatches)
                {
                    originalConstraintCount += typeBatch.ConstraintCount;
                }
            }
            Debug.Assert(constraintHandles.Length == originalConstraintCount);

            //We'll need a mapping from the current handles back to the identity.
            var bodyHandlesToIdentity = new int[simulation.Bodies.HandleToIndex.Length];
            for (int i = 0; i < bodyHandlesToIdentity.Length; ++i)
                bodyHandlesToIdentity[i] = -1;
            var constraintHandlesToIdentity = new int[simulation.Solver.HandleToConstraint.Length];
            for (int i = 0; i < constraintHandlesToIdentity.Length; ++i)
                constraintHandlesToIdentity[i] = -1;

            for (int i = 0; i < bodyHandles.Length; ++i)
            {
                ref var bodyDescription = ref bodyDescriptions[i];
                var handle = bodyHandles[i];
                simulation.Bodies.GetLocalInertia(handle, out bodyDescription.LocalInertia);
                simulation.Bodies.GetVelocity(handle, out bodyDescription.Velocity);
                bodyHandlesToIdentity[handle] = i;
            }

            for (int i = 0; i < constraintHandles.Length; ++i)
            {
                var constraintHandle = constraintHandles[i];
                constraintHandlesToIdentity[constraintHandle] = i;
                simulation.Solver.GetDescription(constraintHandle, out constraintDescriptions[i].Description);
                simulation.Solver.GetConstraintReference(constraintHandle, out var reference);

                var bodyIdentityEnumerator = new BodyEnumerator(simulation.Bodies, bodyHandlesToIdentity);
                reference.TypeBatch.EnumerateConnectedBodyIndices(reference.IndexInTypeBatch, ref bodyIdentityEnumerator);
                constraintDescriptions[i].BodyA = bodyIdentityEnumerator.IdentityA;
                constraintDescriptions[i].BodyB = bodyIdentityEnumerator.IdentityB;
            }



            //Any time a body is removed, the handle in the associated body entry must be updated to -1.
            //All constraints refer to bodies by their out-of-engine identity so that everything stays robust in the face of adds and removes.
            var removedConstraints = new List<int>();
            var removedBodies = new List<int>();
            var random = new Random(5);

            Validate(simulation, removedConstraints, removedBodies, bodyHandles.Length, originalConstraintCount);

            var constraintActionProbability = originalConstraintCount > 0 ? 1 - (double)simulation.Bodies.BodyCount / originalConstraintCount : 0;

            var timer = Stopwatch.StartNew();
            for (int iterationIndex = 0; iterationIndex < iterations; ++iterationIndex)
            {
                if (random.NextDouble() < constraintActionProbability)
                {
                    //Constraint action.
                    var constraintRemovalProbability = (originalConstraintCount - removedConstraints.Count) / (double)originalConstraintCount;
                    if (random.NextDouble() < constraintRemovalProbability)
                    {
                        ChurnRemoveConstraint(simulation, bodyHandles.Length, constraintHandlesToIdentity, constraintHandles, constraintDescriptions, removedConstraints, removedBodies, random);
                    }
                    else if (removedConstraints.Count > 0)
                    {
                        ChurnAddConstraint(simulation, bodyHandles, constraintHandles, constraintHandlesToIdentity, constraintDescriptions, removedConstraints, removedBodies, random);
                    }
                }
                else
                {
                    //Body action.
                    var bodyRemovalProbability = (bodyHandles.Length - removedBodies.Count) / (double)bodyHandles.Length;
                    if (random.NextDouble() < bodyRemovalProbability)
                    {
                        ChurnRemoveBody(simulation, bodyHandles, bodyHandlesToIdentity, constraintHandles, constraintHandlesToIdentity, constraintDescriptions, removedConstraints, removedBodies, random);
                    }
                    else if (removedBodies.Count > 0)
                    {
                        ChurnAddBody(simulation, bodyDescriptions, bodyHandles, bodyHandlesToIdentity, originalConstraintCount, removedConstraints, removedBodies, random);
                    }
                }
            }
            timer.Stop();

            //Go ahead and add everything back so the outer test can proceed unaffected. Theoretically.
            while (removedBodies.Count > 0)
            {
                ChurnAddBody(simulation, bodyDescriptions, bodyHandles, bodyHandlesToIdentity, originalConstraintCount, removedConstraints, removedBodies, random);
            }
            while (removedConstraints.Count > 0)
            {
                ChurnAddConstraint(simulation, bodyHandles, constraintHandles, constraintHandlesToIdentity, constraintDescriptions, removedConstraints, removedBodies, random);
            }

            for (int i = 0; i < constraintHandles.Length; ++i)
            {
                simulation.Solver.GetDescription(constraintHandles[i], out ContactManifold4Constraint description);
                Debug.Assert(description.Equals(constraintDescriptions[i].Description), "Moving constraints around should not affect their descriptions.");
            }

            var newConstraintCount = 0;
            foreach (var batch in simulation.Solver.Batches)
            {
                foreach (var typeBatch in batch.TypeBatches)
                {
                    newConstraintCount += typeBatch.ConstraintCount;
                }
            }
            Debug.Assert(newConstraintCount == originalConstraintCount, "Best have the same number of constraints if we actually added them all back!");
            Debug.Assert(bodyHandles.Length == simulation.Bodies.BodyCount, "And bodies, too!");

            return timer.Elapsed.TotalSeconds;
        }


    }
}

