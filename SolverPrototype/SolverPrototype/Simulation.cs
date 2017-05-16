using BEPUutilities2;
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
    public class Simulation : IDisposable
    {
        public ConstraintConnectivityGraph ConstraintGraph { get; private set; }
        public Bodies Bodies { get; private set; }
        public PoseIntegrator PoseIntegrator { get; private set; }
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
        /// <param name="initialAllocationSizes">Allocation sizes to initialize the simulation with.</param>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        public Simulation(BufferPool bufferPool, SimulationAllocationSizes initialAllocationSizes)
        {
            BufferPool = bufferPool;
            Bodies = new Bodies(bufferPool, initialAllocationSizes.Bodies);
            Solver = new Solver(Bodies, BufferPool, initialCapacity: initialAllocationSizes.Constraints, minimumCapacityPerTypeBatch: initialAllocationSizes.ConstraintsPerTypeBatch);
            ConstraintGraph = new ConstraintConnectivityGraph(Solver, bufferPool, initialAllocationSizes.Bodies, initialAllocationSizes.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer = new BodyLayoutOptimizer(Bodies, ConstraintGraph, Solver, bufferPool);
            ConstraintLayoutOptimizer = new ConstraintLayoutOptimizer(Bodies, Solver);
            SolverBatchCompressor = new BatchCompressor(Solver, Bodies);
        }
        /// <summary>
        /// Constructs a full featured simulation supporting dynamic movement and constraints. Uses a default preallocation size.
        /// </summary>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        public Simulation(BufferPool bufferPool)
            : this(bufferPool, new SimulationAllocationSizes { Bodies = 4096, ConstraintCountPerBodyEstimate = 8, Constraints = 16384, ConstraintsPerTypeBatch = 256 })
        {
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

        //TODO: I wonder if people will abuse the dt-as-parameter to the point where we should make it a field instead, like it effectively was in v1.
        /// <summary>
        /// Performs one timestep of the given length.
        /// </summary>
        /// <remarks>
        /// Be wary of variable timesteps. They can harm stability. Whenever possible, keep the timestep the same across multiple frames unless you have a specific reason not to.
        /// </remarks>
        /// <param name="dt">Duration of the time step in time.</param>
        public void Timestep(float dt, IThreadDispatcher threadDispatcher = null)
        {
            if (threadDispatcher != null)
            {
                //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
                BodyLayoutOptimizer.IncrementalOptimize(BufferPool, threadDispatcher);
                ConstraintLayoutOptimizer.Update(BufferPool, threadDispatcher);
                SolverBatchCompressor.Compress(BufferPool, threadDispatcher);
                PoseIntegrator.Update(dt, threadDispatcher);
                var inverseDt = 1f / dt;
                Solver.MultithreadedUpdate(threadDispatcher, BufferPool, dt, inverseDt);
            }
            else
            {
                BodyLayoutOptimizer.IncrementalOptimize();
                ConstraintLayoutOptimizer.Update(BufferPool);
                SolverBatchCompressor.Compress(BufferPool);
                PoseIntegrator.Update(dt);
                var inverseDt = 1f / dt;
                Solver.Update(dt, inverseDt);
            }
        }

        /// <summary>
        /// Clears the simulation of every object, only returning memory to the pool that would be returned by sequential removes. 
        /// Other persistent allocations, like those in the Bodies set, will remain.
        /// </summary>
        public void Clear()
        {
            ConstraintGraph.Clear(Bodies);
            Solver.Clear();
            Bodies.Clear();
        }
        /// <summary>
        /// Increases the allocation size of any buffers too small to hold the allocation target.
        /// </summary>
        /// <remarks>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </remarks>
        /// <param name="allocationTarget">Allocation sizes to guarantee sufficient size for.</param>
        public void EnsureCapacity(ref SimulationAllocationSizes allocationTarget)
        {
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.EnsureCapacity(allocationTarget.Bodies);
            ConstraintGraph.EnsureCapacity(Bodies, allocationTarget.Bodies, allocationTarget.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);

        }
        /// <summary>
        /// Decreases the size of buffers to the smallest value that still contains the allocation target and any existing objects.
        /// </summary>
        /// <remarks>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </remarks>
        /// <param name="allocationTarget">Target size to compact to. Buffers may be larger to guarantee sufficient room for existing simulation objects.</param>
        public void Compact(ref SimulationAllocationSizes allocationTarget)
        {
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.Compact(allocationTarget.Bodies);
            ConstraintGraph.Compact(Bodies, allocationTarget.Bodies, allocationTarget.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);
        }
        /// <summary>
        /// Increases the allocation size of any buffers too small to hold the allocation target, and decreases the allocation size of any buffers that are unnecessarily large.
        /// </summary>
        /// <remarks>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </remarks>
        /// <param name="allocationTarget">Allocation sizes to guarantee sufficient size for.</param>
        public void Resize(ref SimulationAllocationSizes allocationTarget)
        {
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.Resize(allocationTarget.Bodies);
            ConstraintGraph.Resize(Bodies, allocationTarget.Bodies, allocationTarget.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);
        }
        /// <summary>
        /// Clears the simulation of every object and returns all pooled memory to the buffer pool.
        /// </summary>
        /// <remarks>After disposal, the simulation cannot be used until rehydrated by calling EnsureCapacity or Resize to allocate buffers.</remarks>
        public void Dispose()
        {
            Clear();
            Solver.Dispose(BufferPool);
            Bodies.Dispose();
            BodyLayoutOptimizer.Dispose(BufferPool);
            ConstraintGraph.Dispose();
        }
    }

    /// <summary>
    /// The common set of allocation sizes for a simulation.
    /// </summary>
    public struct SimulationAllocationSizes
    {
        /// <summary>
        /// The number of bodies to allocate space for.
        /// </summary>
        public int Bodies;
        /// <summary>
        /// The number of constraints to allocate bookkeeping space for. This does not affect actual type batch allocation sizes, only the solver-level constraint handle storage.
        /// </summary>
        public int Constraints;
        /// <summary>
        /// The minimum number of constraints to allocate space for in each individual type batch.
        /// New type batches will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
        /// The number of constraints can vary greatly across types- there are usually far more contacts than ragdoll constraints.
        /// Per type estimates can be assigned within the Solver.TypeBatchAllocation if necessary. This value acts as a lower bound for all types.
        /// </summary>
        public int ConstraintsPerTypeBatch;
        /// <summary>
        /// The minimum number of constraints to allocate space for in each body's constraint list.
        /// New bodies will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
        /// </summary>
        public int ConstraintCountPerBodyEstimate;
    }
}
