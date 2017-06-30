using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using SolverPrototype.Constraints;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    /// <summary>
    /// Orchestrates the bookkeeping and execution of a full dynamic simulation.
    /// </summary>
    public partial class Simulation : IDisposable
    {
        public ConstraintConnectivityGraph ConstraintGraph { get; private set; }
        public Bodies Bodies { get; private set; }
        public Shapes Shapes { get; private set; }
        public BodyCollidables BodyCollidables { get; private set; }
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
            Shapes = new Shapes(initialAllocationSizes.ShapesPerType);
            BodyCollidables = new BodyCollidables(Shapes, initialAllocationSizes.CollidablesPerType);
            Solver = new Solver(Bodies, BufferPool,
                initialCapacity: initialAllocationSizes.Constraints,
                minimumCapacityPerTypeBatch: initialAllocationSizes.ConstraintsPerTypeBatch);
            ConstraintGraph = new ConstraintConnectivityGraph(Solver, bufferPool, initialAllocationSizes.Bodies, initialAllocationSizes.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer = new BodyLayoutOptimizer(Bodies, BodyCollidables, ConstraintGraph, Solver, bufferPool);
            ConstraintLayoutOptimizer = new ConstraintLayoutOptimizer(Bodies, Solver);
            SolverBatchCompressor = new BatchCompressor(Solver, Bodies);
            PoseIntegrator = new PoseIntegrator(Bodies, BodyCollidables);
        }
        /// <summary>
        /// Constructs a full featured simulation supporting dynamic movement and constraints. Uses a default preallocation size.
        /// </summary>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        public Simulation(BufferPool bufferPool)
            : this(bufferPool, new SimulationAllocationSizes
            {
                Bodies = 4096,
                ShapesPerType = 128,
                CollidablesPerType = 4096,
                ConstraintCountPerBodyEstimate = 8,
                Constraints = 16384,
                ConstraintsPerTypeBatch = 256
            })
        {
        }


        public int Add(ref BodyDescription bodyDescription)
        {
            var handle = Bodies.Add(ref bodyDescription);
            ConstraintGraph.AddBodyList(Bodies.HandleToIndex[handle]);
            return handle;
        }

        public int Add(ref BodyDescription bodyDescription, ref CollidableDescription collidableDescription)
        {
            var handle = Add(ref bodyDescription);
            if (collidableDescription.ShapeIndex.Exists)
            {
                BodyCollidables.Add(Bodies, Bodies.HandleToIndex[handle], ref collidableDescription, BufferPool);
            }
            return handle;
        }

        /// <summary>
        /// Changes the collidable associated with a body with no restriction on whether the body currently has a collidable or what shape it is.
        /// </summary>
        /// <param name="bodyHandle">Handle of the body to change the collidable of.</param>
        /// <param name="collidableDescription">Description of the collidable to create for the body.</param>
        /// <remarks>This is useful when the body is changing to an entirely different collidable. If you only want to change the settings associated with a body's existing collidable
        /// without changing the type of shape it uses, it is more efficient to just directly access its properties by looking it up in the BodyCollidables.</remarks>
        public void ChangeBodyCollidable(int bodyHandle, ref CollidableDescription collidableDescription)
        {
            Bodies.ValidateExistingHandle(bodyHandle);
            var bodyIndex = Bodies.HandleToIndex[bodyHandle];
            ref var collidableIndex = ref Bodies.Collidables[bodyIndex];
            //Remove the collidable if it already exists.
            if (collidableIndex.Exists)
            {
                BodyCollidables.Remove(Bodies, collidableIndex);
            }
            BodyCollidables.Add(Bodies, bodyIndex, ref collidableDescription, BufferPool);
        }


        public void RemoveBody(int bodyHandle)
        {
            Bodies.ValidateExistingHandle(bodyHandle);

            var bodyIndex = Bodies.HandleToIndex[bodyHandle];
            var collidableIndex = Bodies.Collidables[bodyIndex];
            if (Bodies.RemoveAt(bodyIndex, out var movedBodyOriginalIndex))
            {
                //While the removed body doesn't have any constraints associated with it, the body that gets moved to fill its slot might!
                //We're borrowing the body optimizer's logic here. You could share a bit more- the body layout optimizer has to deal with the same stuff, though it's optimized for swaps.
                BodyLayoutOptimizer.UpdateForBodyMemoryMove(movedBodyOriginalIndex, bodyIndex, Bodies, BodyCollidables, ConstraintGraph, Solver);
            }
            //Not every body is forced to have a collidable. Only try to remove those that exist!
            if (collidableIndex.Exists)
            {
                BodyCollidables.Remove(Bodies, collidableIndex);
            }

            var constraintListWasEmpty = ConstraintGraph.RemoveBodyList(bodyIndex, movedBodyOriginalIndex);
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
            ProfilerClear();
            ProfilerStart(this);
            if (threadDispatcher != null)
            {
                //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.

                ProfilerStart(BodyLayoutOptimizer);
                BodyLayoutOptimizer.IncrementalOptimize(BufferPool, threadDispatcher);
                ProfilerEnd(BodyLayoutOptimizer);

                ProfilerStart(ConstraintLayoutOptimizer);
                ConstraintLayoutOptimizer.Update(BufferPool, threadDispatcher);
                ProfilerEnd(ConstraintLayoutOptimizer);

                ProfilerStart(SolverBatchCompressor);
                SolverBatchCompressor.Compress(BufferPool, threadDispatcher);
                ProfilerEnd(SolverBatchCompressor);

                ProfilerStart(PoseIntegrator);
                PoseIntegrator.Update(dt, BufferPool, threadDispatcher);
                ProfilerEnd(PoseIntegrator);

                ProfilerStart(Solver);
                Solver.MultithreadedUpdate(threadDispatcher, BufferPool, dt);
                ProfilerEnd(Solver);
            }
            else
            {
                ProfilerStart(BodyLayoutOptimizer);
                BodyLayoutOptimizer.IncrementalOptimize();
                ProfilerEnd(BodyLayoutOptimizer);

                ProfilerStart(ConstraintLayoutOptimizer);
                ConstraintLayoutOptimizer.Update(BufferPool);
                ProfilerEnd(ConstraintLayoutOptimizer);

                ProfilerStart(SolverBatchCompressor);
                SolverBatchCompressor.Compress(BufferPool);
                ProfilerEnd(SolverBatchCompressor);

                ProfilerStart(PoseIntegrator);
                PoseIntegrator.Update(dt, BufferPool);
                ProfilerEnd(PoseIntegrator);

                ProfilerStart(Solver);
                Solver.Update(dt);
                ProfilerEnd(Solver);
            }
            ProfilerEnd(this);
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
            //TODO: shapes/collidables
        }
        /// <summary>
        /// Increases the allocation size of any buffers too small to hold the allocation target.
        /// </summary>
        /// <remarks>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </remarks>
        /// <param name="allocationTarget">Allocation sizes to guarantee sufficient size for.</param>
        public void EnsureCapacity(SimulationAllocationSizes allocationTarget)
        {
            Solver.EnsureCapacity(allocationTarget.Bodies, allocationTarget.Constraints, allocationTarget.ConstraintsPerTypeBatch);
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.EnsureCapacity(allocationTarget.Bodies);
            ConstraintGraph.EnsureCapacity(Bodies, allocationTarget.Bodies, allocationTarget.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);
            //TODO: shapes/collidables

        }
        /// <summary>
        /// Decreases the size of buffers to the smallest value that still contains the allocation target and any existing objects.
        /// </summary>
        /// <remarks>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </remarks>
        /// <param name="allocationTarget">Target size to compact to. Buffers may be larger to guarantee sufficient room for existing simulation objects.</param>
        public void Compact(SimulationAllocationSizes allocationTarget)
        {
            Solver.Compact(allocationTarget.Bodies, allocationTarget.Constraints, allocationTarget.ConstraintsPerTypeBatch);
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.Compact(allocationTarget.Bodies);
            ConstraintGraph.Compact(Bodies, allocationTarget.Bodies, allocationTarget.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);
            //TODO: shapes/collidables
        }
        /// <summary>
        /// Increases the allocation size of any buffers too small to hold the allocation target, and decreases the allocation size of any buffers that are unnecessarily large.
        /// </summary>
        /// <remarks>
        /// The final size of the allocated buffers are constrained by the allocator. It is not guaranteed to be exactly equal to the target, but it is guaranteed to be at least as large.
        /// </remarks>
        /// <param name="allocationTarget">Allocation sizes to guarantee sufficient size for.</param>
        public void Resize(SimulationAllocationSizes allocationTarget)
        {
            Solver.Resize(allocationTarget.Bodies, allocationTarget.Constraints, allocationTarget.ConstraintsPerTypeBatch);
            //Note that the bodies set has to come before the body layout optimizer; the body layout optimizer's sizes are dependent upon the bodies set.
            Bodies.Resize(allocationTarget.Bodies);
            ConstraintGraph.Resize(Bodies, allocationTarget.Bodies, allocationTarget.ConstraintCountPerBodyEstimate);
            BodyLayoutOptimizer.ResizeForBodiesCapacity(BufferPool);
            //TODO: shapes/collidables
        }
        /// <summary>
        /// Clears the simulation of every object and returns all pooled memory to the buffer pool.
        /// </summary>
        /// <remarks>After disposal, the simulation cannot be used until rehydrated by calling EnsureCapacity or Resize to allocate buffers.</remarks>
        public void Dispose()
        {
            Clear();
            Solver.Dispose();
            Bodies.Dispose();
            BodyLayoutOptimizer.Dispose(BufferPool);
            ConstraintGraph.Dispose();
            //TODO: shapes/collidables
        }
    }
}
