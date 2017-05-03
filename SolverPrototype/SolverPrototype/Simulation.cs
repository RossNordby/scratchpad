using BEPUutilities2.Memory;
using SolverPrototype.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace SolverPrototype
{
    /// <summary>
    /// Orchestrates the bookkeeping and execution of a full dynamic simulation.
    /// </summary>
    public class Simulation
    {
        public ConstraintConnectivityGraph ConstraintGraph { get; private set; }
        public Bodies Bodies { get; private set; }
        public BodyLayoutOptimizer BodyLayoutOptimizer { get; private set; }
        public ConstraintLayoutOptimizer ConstraintLayoutOptimizer { get; private set; }
        public BatchCompressor SolverBatchCompressor { get; private set; }
        public Solver Solver { get; private set; }

        /// <summary>
        /// Gets the main memory pool used to fill persistent structures and main thread ephemeral resources across the engine.
        /// </summary>
        public BufferPool BufferPool { get; private set; }

        //You might wonder why this calls out 'full featured', since there are no other non-full featured simulations.
        //Later on, we might include such a partial simulation- for example, kinematic only, or even collision detection only.
        //The only real difference is what stages are included (and a few changes in memory layout).
        /// <summary>
        /// Constructs a full featured simulation supporting dynamic movement and constraints.
        /// </summary>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        /// <param name="initialBodyCapacity">Expected number of bodies. If the simulation exceeds this number, a resize of relevant resources will be performed.</param>
        /// <param name="initialConstraintCapacity">Expected number of constraints across all types. If the simulation exceeds this number, a resize of relevant resources will be performed.</param>
        /// <param name="minimumCapacityPerTypeBatch">Minimum number of constraints of one type in a constraint batch.
        /// This can vary greatly across types- there are usually far more contacts than ragdoll constraints.
        /// Per type estimates can be assigned within the Solver.TypeBatchAllocation.</param>
        /// <param name="initialConstraintCountPerBodyEstimate">Expected number of constraints connected to any one body. If this number is exceeded for a body, a resize of the body's list will be performed, but all affected resources are pooled.</param>
        public Simulation(
            BufferPool bufferPool,
            int initialBodyCapacity = 4096,
            int initialConstraintCapacity = 4096,
            int minimumCapacityPerTypeBatch = 64,
            int initialConstraintCountPerBodyEstimate = 8)
        {
            BufferPool = bufferPool;
            Bodies = new Bodies(initialBodyCapacity);
            Solver = new Solver(Bodies, BufferPool, initialCapacity: initialConstraintCapacity, minimumCapacityPerTypeBatch: minimumCapacityPerTypeBatch);
            ConstraintGraph = new ConstraintConnectivityGraph(Solver, bufferPool, initialBodyCapacity, initialConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer = new BodyLayoutOptimizer(Bodies, ConstraintGraph, Solver, bufferPool);
            ConstraintLayoutOptimizer = new ConstraintLayoutOptimizer(Bodies, Solver);
            SolverBatchCompressor = new BatchCompressor(Solver, Bodies);
        }

        public int Add(ref BodyDescription bodyDescription)
        {
            var handle = Bodies.Add(ref bodyDescription);
            ConstraintGraph.AddBodyList(Bodies.HandleToIndex[handle]);
            return handle;
        }


        public void RemoveBody(int bodyHandle)
        {
            Bodies.ValidateExistingHandle(bodyHandle);

            if (Bodies.Remove(bodyHandle, out var removedIndex, out var movedBodyOriginalIndex))
            {
                //While the removed body doesn't have any constraints associated with it, the body that gets moved to fill its slot might!
                //We're borrowing the body optimizer's logic here. You could share a bit more- the body layout optimizer has to deal with the same stuff, though it's optimized for swaps.
                BodyLayoutOptimizer.UpdateConstraintsForBodyMemoryMove(movedBodyOriginalIndex, removedIndex, ConstraintGraph, Solver);
            }

            var constraintListWasEmpty = ConstraintGraph.RemoveBodyList(removedIndex, movedBodyOriginalIndex);
            Debug.Assert(constraintListWasEmpty, "Removing a body without first removing its constraints results in orphaned constraints that will break stuff. Don't do it!");


        }

        /// <summary>
        /// Allocates a constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandles">First body handle in a list of body handles used by the constraint.</param>
        /// <param name="bodyCount">Number of bodies used by the constraint.</param>
        /// <returns>Allocated constraint handle.</returns>
        public int Add<TDescription>(ref int bodyHandles, int bodyCount, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            Solver.Add(ref bodyHandles, bodyCount, ref description, out int constraintHandle);
            for (int i = 0; i < bodyCount; ++i)
            {
                Bodies.ValidateExistingHandle(Unsafe.Add(ref bodyHandles, i));
                ConstraintGraph.AddConstraint(Bodies.HandleToIndex[Unsafe.Add(ref bodyHandles, i)], constraintHandle, i);
            }
            return constraintHandle;
        }

        /// <summary>
        /// Allocates a two-body constraint slot and sets up a constraint with the specified description.
        /// </summary>
        /// <typeparam name="TDescription">Type of the constraint description to add.</typeparam>
        /// <param name="bodyHandleA">First body of the pair.</param>
        /// <param name="bodyHandleB">Second body of the pair.</param>
        /// <returns>Allocated constraint handle.</returns>
        public unsafe int Add<TDescription>(int bodyHandleA, int bodyHandleB, ref TDescription description)
            where TDescription : IConstraintDescription<TDescription>
        {
            //Don't really want to take a dependency on the stack layout of parameters, so...
            var bodyReferences = stackalloc int[2];
            bodyReferences[0] = bodyHandleA;
            bodyReferences[1] = bodyHandleB;
            return Add(ref bodyReferences[0], 2, ref description);
        }

        struct ConstraintGraphRemovalEnumerator : IForEach<int>
        {
            internal ConstraintConnectivityGraph graph;
            internal int constraintHandle;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void LoopBody(int bodyIndex)
            {
                graph.RemoveConstraint(bodyIndex, constraintHandle);
            }
        }
        public void RemoveConstraint(int constraintHandle)
        {
            ConstraintGraphRemovalEnumerator enumerator;
            enumerator.graph = ConstraintGraph;
            enumerator.constraintHandle = constraintHandle;
            Solver.EnumerateConnectedBodyIndices(constraintHandle, ref enumerator);
            Solver.Remove(constraintHandle);
        }

        /// <summary>
        /// Performs one timestep of the given length.
        /// </summary>
        /// <remarks>
        /// Be wary of variable timesteps. They can harm stability. Whenever possible, keep the timestep the same across multiple frames unless you have a specific reason not to.
        /// </remarks>
        /// <param name="dt">Duration of the time step in time.</param>
        public void Timestep(float dt)
        {
            var inverseDt = 1f / dt;
            Solver.Update(dt, inverseDt);
        }
    }
}
