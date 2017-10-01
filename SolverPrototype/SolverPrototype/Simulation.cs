using BEPUutilities2;
using BEPUutilities2.Memory;
using SolverPrototype.Collidables;
using SolverPrototype.CollisionDetection;
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
        public BodyLayoutOptimizer BodyLayoutOptimizer { get; private set; }
        public ConstraintLayoutOptimizer ConstraintLayoutOptimizer { get; private set; }
        public BatchCompressor SolverBatchCompressor { get; private set; }
        public Solver Solver { get; private set; }
        public PoseIntegrator PoseIntegrator { get; private set; }
        public BroadPhase BroadPhase { get; private set; }
        public CollidableOverlapFinder BroadPhaseOverlapFinder { get; private set; }
        public NarrowPhase NarrowPhase { get; private set; }



        /// <summary>
        /// Gets the main memory pool used to fill persistent structures and main thread ephemeral resources across the engine.
        /// </summary>
        public BufferPool BufferPool { get; private set; }

        protected Simulation(BufferPool bufferPool, SimulationAllocationSizes initialAllocationSizes)
        {
            BufferPool = bufferPool;
            Bodies = new Bodies(bufferPool, initialAllocationSizes.Bodies);
            Shapes = new Shapes(bufferPool, initialAllocationSizes.ShapesPerType);
            Solver = new Solver(Bodies, BufferPool,
                initialCapacity: initialAllocationSizes.Constraints,
                minimumCapacityPerTypeBatch: initialAllocationSizes.ConstraintsPerTypeBatch);
            ConstraintGraph = new ConstraintConnectivityGraph(Solver, bufferPool, initialAllocationSizes.Bodies, initialAllocationSizes.ConstraintCountPerBodyEstimate);

            BroadPhase = new BroadPhase(bufferPool, initialAllocationSizes.Bodies);
            PoseIntegrator = new PoseIntegrator(Bodies, Shapes, BroadPhase);

            SolverBatchCompressor = new BatchCompressor(Solver, Bodies);
            BodyLayoutOptimizer = new BodyLayoutOptimizer(Bodies, BroadPhase, ConstraintGraph, Solver, bufferPool);
            ConstraintLayoutOptimizer = new ConstraintLayoutOptimizer(Bodies, Solver);
        }

        /// <summary>
        /// Constructs a simulation supporting dynamic movement and constraints with the specified narrow phase callbacks.
        /// </summary>
        /// <param name="bufferPool">Buffer pool used to fill persistent structures and main thread ephemeral resources across the engine.</param>
        /// <param name="narrowPhaseCallbacks">Callbacks to use in the narrow phase.</param>
        /// <param name="initialAllocationSizes">Allocation sizes to initialize the simulation with. If left null, default values are chosen.</param>
        /// <param name="collisionTaskRegistry">The set of collision tasks to use in the simulation. If left null, default types are registered.</param>
        /// <returns>New simulation.</returns>
        public static Simulation Create<TNarrowPhaseCallbacks>(BufferPool bufferPool, TNarrowPhaseCallbacks narrowPhaseCallbacks,
            SimulationAllocationSizes? initialAllocationSizes = null, CollisionTaskRegistry collisionTaskRegistry = null)
            where TNarrowPhaseCallbacks : struct, INarrowPhaseCallbacks
        {
            if (initialAllocationSizes == null)
            {
                initialAllocationSizes = new SimulationAllocationSizes
                {
                    Bodies = 4096,
                    ShapesPerType = 128,
                    CollidablesPerType = 4096,
                    ConstraintCountPerBodyEstimate = 8,
                    Constraints = 16384,
                    ConstraintsPerTypeBatch = 256
                };
            }
            if (collisionTaskRegistry == null)
            {
                collisionTaskRegistry = DefaultTypes.CreateDefaultCollisionTaskRegistry();
            }
            var simulation = new Simulation(bufferPool, initialAllocationSizes.Value);
            var narrowPhase = new NarrowPhase<TNarrowPhaseCallbacks>(simulation, collisionTaskRegistry, narrowPhaseCallbacks);
            simulation.NarrowPhase = narrowPhase;
            simulation.BroadPhaseOverlapFinder = new CollidableOverlapFinder<TNarrowPhaseCallbacks>(narrowPhase, simulation.BroadPhase);

            return simulation;
        }


        void AddCollidableToBroadPhase(int bodyHandle, ref BodyDescription bodyDescription, ref Collidable collidable)
        {
            //This body has a collidable; stick it in the broadphase.
            //Note that we have to calculate an initial bounding box for the broad phase to be able to insert it efficiently.
            //(In the event of batch adds, you'll want to use batched AABB calculations or just use cached values.)
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            BoundingBox bodyBounds;
            Shapes[collidable.Shape.Type].ComputeBounds(collidable.Shape.Index, ref bodyDescription.Pose, out bodyBounds.Min, out bodyBounds.Max);
            collidable.BroadPhaseIndex =
                BroadPhase.Add(new CollidableReference(bodyDescription.Mobility, bodyHandle), ref bodyBounds);

        }

        void RemoveCollidableFromBroadPhase(ref Collidable collidable)
        {
            var removedBroadPhaseIndex = collidable.BroadPhaseIndex;
            if (BroadPhase.RemoveAt(removedBroadPhaseIndex, out var movedLeaf))
            {
                //When a leaf is removed from the broad phase, another leaf will move to take its place in the leaf set.
                //We must update the collidable->leaf index pointer to match the new position of the leaf in the broadphase.
                //There are three possible cases for the moved leaf:
                //1) it is an active body collidable,
                //2) it is an inactive body collidable,
                //3) it is a static collidable.
                //The collidable reference we retrieved tells us whether it's a body or a static.
                //In the event that it's a body, we can infer the activity state from the body we just removed. Any body within the same 'leaf space' as the removed body
                //shares its activity state. This involves some significant conceptual coupling with the broad phase's implementation, but that's a price we're willing to pay
                //if it avoids extraneous data storage.
                //TODO: When deactivation exists, this will need to be updated to accommodate it.
                //TODO: When non-body static collidables exist, this will need to be updated.
                Bodies.Collidables[Bodies.HandleToIndex[movedLeaf.Handle]].BroadPhaseIndex = removedBroadPhaseIndex;
            }
        }

        public int Add(ref BodyDescription bodyDescription)
        {
            var handle = Bodies.Add(ref bodyDescription);
            var bodyIndex = Bodies.HandleToIndex[handle];
            ConstraintGraph.AddBodyList(bodyIndex);
            if (bodyDescription.Collidable.Shape.Exists)
            {
                AddCollidableToBroadPhase(handle, ref bodyDescription, ref Bodies.Collidables[bodyIndex]);
            }
            return handle;
        }

        public void ApplyDescription(int handle, ref BodyDescription bodyDescription)
        {
            Bodies.ValidateExistingHandle(handle);
            var bodyIndex = Bodies.HandleToIndex[handle];
            ref var collidable = ref Bodies.Collidables[bodyIndex];
            var broadPhaseUpdateRequired = collidable.Shape.Exists != bodyDescription.Collidable.Shape.Exists;
            Bodies.SetDescriptionByIndex(bodyIndex, ref bodyDescription);
            if (broadPhaseUpdateRequired)
            {
                //A collidable has been added or removed by this description change. Which is it?
                if (bodyDescription.Collidable.Shape.Exists)
                {
                    //Adding!               
                    AddCollidableToBroadPhase(handle, ref bodyDescription, ref collidable);
                }
                else
                {
                    //Removing!
                    RemoveCollidableFromBroadPhase(ref collidable);
                }
            }
            else
            {
                //While we aren't adding or removing a collidable, we may be changing the mobility.
                if (bodyDescription.Collidable.Shape.Exists)
                    BroadPhase.activeLeaves[collidable.BroadPhaseIndex] = new CollidableReference(bodyDescription.Mobility, handle);
            }
        }
        
        public void RemoveBody(int bodyHandle)
        {
            Bodies.ValidateExistingHandle(bodyHandle);

            var bodyIndex = Bodies.HandleToIndex[bodyHandle];
            ref var collidable = ref Bodies.Collidables[bodyIndex];
            if (collidable.Shape.Exists)
            {
                //The collidable exists, so it should be removed from the broadphase.
                RemoveCollidableFromBroadPhase(ref collidable);
            }
            if (Bodies.RemoveAt(bodyIndex, out var movedBodyOriginalIndex))
            {
                //While the removed body doesn't have any constraints associated with it, the body that gets moved to fill its slot might!
                //We're borrowing the body optimizer's logic here. You could share a bit more- the body layout optimizer has to deal with the same stuff, though it's optimized for swaps.
                BodyLayoutOptimizer.UpdateForBodyMemoryMove(movedBodyOriginalIndex, bodyIndex, Bodies, ConstraintGraph, Solver);
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
            //TODO: There are a couple of stages where the multithreaded and single threaded implementations differ by more than merely a null thread dispatcher.
            //All other stages internally handle the availability of threading in an case-by-case way. It would be nice if every stage did so so we didn't need a dual implementation out here.
            if (threadDispatcher != null)
            {

                //Note that the first behavior-affecting stage is actually the pose integrator. This is a shift from v1, where collision detection went first.
                //This is a tradeoff:
                //1) Any externally set velocities will be integrated without input from the solver. The v1-style external velocity control won't work as well-
                //the user would instead have to change velocities after the pose integrator runs. This isn't perfect either, since the pose integrator is also responsible
                //for updating the bounding boxes used for collision detection.
                //2) By bundling bounding box calculation with pose integration, you avoid redundant pose and velocity memory accesses.
                //3) Generated contact positions are in sync with the integrated poses. 
                //That's often helpful for gameplay purposes- you don't have to reinterpret contact data when creating graphical effects or positioning sound sources.

                //TODO: This is something that is possibly worth exposing as one of the generic type parameters. Users could just choose the order arbitrarily.
                //Or, since you're talking about something that happens once per frame instead of once per collision pair, just provide a simple callback.
                //(Or maybe an enum even?)
                //#1 is a difficult problem, though. There is no fully 'correct' place to change velocities. We might just have to bite the bullet and create a
                //inertia tensor/bounding box update separate from pose integration. If the cache gets evicted in between (virtually guaranteed unless no stages run),
                //this basically means an extra 100-200 microseconds per frame on a processor with ~20GBps bandwidth simulating 32768 bodies.

                //Note that the reason why the pose integrator comes first instead of, say, the solver, is that the solver relies on world space inertias calculated by the pose integration.
                //If the pose integrator doesn't run first, we either need 
                //1) complicated on demand updates of world inertia when objects are added or local inertias are changed or 
                //2) local->world inertia calculation before the solver.                

                ProfilerStart(PoseIntegrator);
                PoseIntegrator.Update(dt, BufferPool, threadDispatcher);
                ProfilerEnd(PoseIntegrator);

                ProfilerStart(BroadPhase);
                BroadPhase.Update(threadDispatcher);
                ProfilerEnd(BroadPhase);

                ProfilerStart(BroadPhaseOverlapFinder);
                BroadPhaseOverlapFinder.DispatchOverlaps(threadDispatcher);
                ProfilerEnd(BroadPhaseOverlapFinder);

                ProfilerStart(NarrowPhase);
                NarrowPhase.Flush(threadDispatcher);
                ProfilerEnd(NarrowPhase);

                ProfilerStart(Solver);
                Solver.MultithreadedUpdate(threadDispatcher, BufferPool, dt);
                ProfilerEnd(Solver);

                //Note that constraint optimization should be performed after body optimization, since body optimization moves the bodies- and so affects the optimal constraint position.
                //TODO: The order of these optimizer stages is performance relevant, even though they don't have any effect on correctness.
                //You may want to try them in different locations to see how they impact cache residency.
                ProfilerStart(BodyLayoutOptimizer);
                BodyLayoutOptimizer.IncrementalOptimize(BufferPool, threadDispatcher);
                ProfilerEnd(BodyLayoutOptimizer);

                ProfilerStart(ConstraintLayoutOptimizer);
                ConstraintLayoutOptimizer.Update(BufferPool, threadDispatcher);
                ProfilerEnd(ConstraintLayoutOptimizer);

                ProfilerStart(SolverBatchCompressor);
                SolverBatchCompressor.Compress(BufferPool, threadDispatcher);
                ProfilerEnd(SolverBatchCompressor);

            }
            else
            {
                ProfilerStart(PoseIntegrator);
                PoseIntegrator.Update(dt, BufferPool);
                ProfilerEnd(PoseIntegrator);

                ProfilerStart(BroadPhase);
                BroadPhase.Update();
                ProfilerEnd(BroadPhase);
                
                ProfilerStart(BroadPhaseOverlapFinder);
                BroadPhaseOverlapFinder.DispatchOverlaps();
                ProfilerEnd(BroadPhaseOverlapFinder);

                ProfilerStart(NarrowPhase);
                NarrowPhase.Flush();
                ProfilerEnd(NarrowPhase);

                ProfilerStart(Solver);
                Solver.Update(dt);
                ProfilerEnd(Solver);

                ProfilerStart(BodyLayoutOptimizer);
                BodyLayoutOptimizer.IncrementalOptimize();
                ProfilerEnd(BodyLayoutOptimizer);

                ProfilerStart(ConstraintLayoutOptimizer);
                ConstraintLayoutOptimizer.Update(BufferPool);
                ProfilerEnd(ConstraintLayoutOptimizer);

                ProfilerStart(SolverBatchCompressor);
                SolverBatchCompressor.Compress(BufferPool);
                ProfilerEnd(SolverBatchCompressor);
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
            //TODO: shapes/broadphase
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
            //TODO: shapes/broadphase

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
            //TODO: shapes/broadphase
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
            //TODO: shapes/broadphase
        }
        /// <summary>
        /// Clears the simulation of every object and returns all pooled memory to the buffer pool.
        /// </summary>
        /// <remarks>After disposal, the simulation cannot be used until rehydrated by calling EnsureCapacity or Resize to allocate buffers.</remarks>
        public void Dispose()
        {
            Clear();
            Solver.Dispose();
            BroadPhase.Dispose();
            NarrowPhase.Dispose();
            Bodies.Dispose();
            BodyLayoutOptimizer.Dispose(BufferPool);
            ConstraintGraph.Dispose();
            //TODO: shapes/broadphase
        }
    }
}
