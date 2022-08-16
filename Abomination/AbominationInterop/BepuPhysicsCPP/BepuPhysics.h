#pragma once

#include <stdint.h>
#include <assert.h>
#include "InteropMath.h"
#include "Bodies.h"
#include "Statics.h"
#include "Utilities.h"
#include "Collisions.h"
#include "PoseIntegration.h"
#include "Shapes.h"

namespace Bepu
{
	enum SIMDWidth { SIMD128, SIMD256, SIMD512 };

	/// <summary>
	/// Defines properties of the solver 
	/// </summary>
	struct SolveDescription
	{
		/// <summary>
		/// Number of velocity iterations to use in the solver if there is no <see cref="VelocityIterationScheduler"/> or if it returns a non-positive value for a substep.
		/// </summary>
		int32_t VelocityIterationCount;
		/// <summary>
		/// Number of substeps to execute each time the solver runs.
		/// </summary>
		int32_t SubstepCount;
		/// <summary>
		/// Number of synchronzed constraint batches to use before using a fallback approach.
		/// </summary>
		int32_t FallbackBatchThreshold;
		/// <summary>
		/// Callback executed to determine how many velocity iterations should be used for a given substep. If null, or if it returns a non-positive value, the <see cref="VelocityIterationCount"/> will be used instead.
		/// </summary>	
		/// <param name="substepIndex">Index of the substep to schedule velocity iterations for.</param>
		/// <returns>Number of velocity iterations to run during this substep.</returns>
		int32_t(*VelocityIterationScheduler)(int32_t substepIndex);

		/// <summary>
		/// Creates a solve description.
		/// </summary>
		/// <param name="velocityIterationCount">Number of velocity iterations per substep.</param>
		/// <param name="substepCount">Number of substeps in the solve.</param>
		/// <param name="fallbackBatchThreshold">Number of synchronzed constraint batches to use before using a fallback approach.</param>
		SolveDescription(int velocityIterationCount, int substepCount, int fallbackBatchThreshold = 64)
		{
			VelocityIterationCount = velocityIterationCount;
			SubstepCount = substepCount;
			FallbackBatchThreshold = fallbackBatchThreshold;
			VelocityIterationScheduler = NULL;
		}
	};

	/// <summary>
	/// The common set of allocation sizes for a simulation.
	/// </summary>
	struct SimulationAllocationSizes
	{
		/// <summary>
		/// The number of bodies to allocate space for.
		/// </summary>
		int32_t Bodies;
		/// <summary>
		/// The number of statics to allocate space for.
		/// </summary>
		int32_t Statics;
		/// <summary>
		/// The number of inactive islands to allocate space for.
		/// </summary>
		int32_t Islands;
		/// <summary>
		/// Minimum number of shapes to allocate space for in each shape type batch.
		/// </summary>
		int32_t ShapesPerType;
		/// <summary>
		/// The number of constraints to allocate bookkeeping space for. This does not affect actual type batch allocation sizes, only the solver-level constraint handle storage.
		/// </summary>
		int32_t Constraints;
		/// <summary>
		/// The minimum number of constraints to allocate space for in each individual type batch.
		/// New type batches will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
		/// The number of constraints can vary greatly across types- there are usually far more contacts than ragdoll constraints.
		/// Per type estimates can be assigned within the Solver.TypeBatchAllocation if necessary. This value acts as a lower bound for all types.
		/// </summary>
		int32_t ConstraintsPerTypeBatch;
		/// <summary>
		/// The minimum number of constraints to allocate space for in each body's constraint list.
		/// New bodies will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
		/// </summary>
		int32_t ConstraintCountPerBodyEstimate;

		/// <summary>
		/// Constructs a description of simulation allocations.
		/// </summary>
		/// <param name="bodies">The number of bodies to allocate space for.</param>
		/// <param name="statics">The number of statics to allocate space for.</param>
		/// <param name="islands">The number of inactive islands to allocate space for.</param>
		/// <param name="shapesPerType">Minimum number of shapes to allocate space for in each shape type batch.</param>
		/// <param name="constraints">The number of constraints to allocate bookkeeping space for. This does not affect actual type batch allocation sizes, only the solver-level constraint handle storage.</param>
		/// <param name="constraintsPerTypeBatch">The minimum number of constraints to allocate space for in each individual type batch.
		/// New type batches will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
		/// The number of constraints can vary greatly across types- there are usually far more contacts than ragdoll constraints.
		/// Per type estimates can be assigned within the Solver.TypeBatchAllocation if necessary. This value acts as a lower bound for all types.</param>
		/// <param name="constraintCountPerBodyEstimate">The minimum number of constraints to allocate space for in each body's constraint list.
		/// New bodies will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.</param>
		SimulationAllocationSizes(int32_t bodies = 4096, int32_t statics = 4096, int32_t islands = 16, int32_t shapesPerType = 128, int32_t constraints = 16384, int32_t constraintsPerTypeBatch = 256, int32_t constraintCountPerBodyEstimate = 8)
		{
			Bodies = bodies;
			Statics = statics;
			Islands = islands;
			ShapesPerType = shapesPerType;
			Constraints = constraints;
			ConstraintsPerTypeBatch = constraintsPerTypeBatch;
			ConstraintCountPerBodyEstimate = constraintCountPerBodyEstimate;
		}
	};

	/// <summary>
	/// Location of a body in memory.
	/// </summary>
	struct BodyMemoryLocation
	{
		/// <summary>
		/// Index of the set owning the body reference. If the set index is 0, the body is awake. If the set index is greater than zero, the body is asleep.
		/// </summary>
		int32_t SetIndex;
		/// <summary>
		/// Index of the body within its owning set.
		/// </summary>
		int32_t Index;
	};

	/// <summary>
	/// Stores a group of bodies- either the set of active bodies, or the bodies involved in an inactive simulation island.
	/// </summary>
	struct BodySet
	{
		/// <summary>
		/// Remaps a body index to its handle.
		/// </summary>
		Buffer<BodyHandle> IndexToHandle;

		/// <summary>
		/// Stores all data involved in solving constraints for a body, including pose, velocity, and inertia.
		/// </summary>
		Buffer<BodyDynamics> DynamicsState;

		/// <summary>
		/// The collidables owned by each body in the set. Speculative margins, continuity settings, and shape indices can be changed directly.
		/// Shape indices cannot transition between pointing at a shape and pointing at nothing or vice versa without notifying the broad phase of the collidable addition or removal.
		/// </summary>
		Buffer<Collidable> Collidables;
		/// <summary>
		/// Activity states of bodies in the set.
		/// </summary>
		Buffer<BodyActivity> Activity;
		/// <summary>
		/// List of constraints associated with each body in the set.
		/// </summary>
		Buffer<QuickList<BodyConstraintReference>> Constraints;

		/// <summary>
		/// Number of bodies in the body set.
		/// </summary>
		int32_t Count;
		/// <summary>
		/// Gets whether this instance is backed by allocated memory.
		/// </summary>
		bool IsAllocated() { return IndexToHandle.Memory != nullptr; }
	};

	/// <summary>
/// Gets a pointer to a big compound shape's data stored within the simulation's shapes buffers.
/// </summary>
/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
/// <param name="shape">Shape reference to request from the simulation.</param>
/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" BigCompound * GetBigCompoundShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Gets a pointer to a mesh shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" Mesh * GetMeshShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Initializes the interop structures.
	/// </summary>
	extern "C" void Initialize();
	/// <summary>
	/// Destroys all resources created through the interop API and releases interop structures.
	/// </summary>
	extern "C" void Destroy();
	extern "C" SIMDWidth GetSIMDWidth();
	/// <summary>
	/// Gets the number of threads exposed by the operating system on this platform. Cores with SMT can show as having multiple threads.
	/// </summary>
	/// <returns>Number of threads exposed by the operating system on this platform.</returns>
	extern "C" int32_t GetPlatformThreadCount();
	/// <summary>
	/// Creates a new buffer pool.
	/// </summary>
	/// <param name="minimumBlockAllocationSize">Minimum size of individual block allocations. Must be a power of 2.
	/// Pools with single allocations larger than the minimum will use the minimum value necessary to hold one element.
	/// Buffers will be suballocated from blocks.</param>
	/// <param name="expectedUsedSlotCountPerPool">Number of suballocations to preallocate reference space for.
	/// This does not preallocate actual blocks, just the space to hold references that are waiting in the pool.</param>
	extern "C" BufferPoolHandle CreateBufferPool(int32_t minimumBlockAllocationSize = 131072, int32_t expectedUsedSlotCountPerPool = 16);
	/// <summary>
	/// Releases all allocations held by the buffer pool. The buffer pool remains in a usable state.
	/// </summary>
	/// <param name="handle">Buffer pool to clear.</param>
	extern "C" void ClearBufferPool(BufferPoolHandle handle);
	/// <summary>
	/// Releases all allocations held by the buffer pool and releases the buffer pool reference. The handle is invalidated.
	/// </summary>
	/// <param name="handle">Buffer pool to destroy.</param>
	extern "C" void DestroyBufferPool(BufferPoolHandle handle);
	/// <summary>
	/// Allocates a buffer from the buffer pool of the given size.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
	/// <param name="sizeInBytes">Size of the buffer to allocate in bytes.</param>
	/// <returns>Allocated buffer.</returns>
	extern "C" ByteBuffer Allocate(BufferPoolHandle bufferPoolHandle, int32_t sizeInBytes);
	/// <summary>
	/// Allocates a buffer from the buffer pool with at least the given size.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
	/// <param name="sizeInBytes">Size of the buffer to allocate in bytes.</param>
	/// <returns>Allocated buffer.</returns>
	extern "C" ByteBuffer AllocateAtLeast(BufferPoolHandle bufferPoolHandle, int32_t sizeInBytes);
	/// <summary>
	/// Resizes a buffer from the buffer pool to the given size, reallocating if necessary.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
	/// <param name="buffer">Buffer to resize.</param>
	/// <param name="newSizeInBytes">Target size of the buffer to allocate in bytes.</param>
	/// <param name="copyCount">Number of bytes to copy from the old buffer into the new buffer.</param>
	extern "C" void Resize(BufferPoolHandle bufferPoolHandle, ByteBuffer * buffer, int32_t newSizeInBytes, int32_t copyCount);
	/// <summary>
	/// Resizes a buffer from the buffer pool to at least the given size, reallocating if necessary.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to allocate from.</param>
	/// <param name="buffer">Buffer to resize.</param>
	/// <param name="targetSizeInBytes">Target size of the buffer to allocate in bytes.</param>
	/// <param name="copyCount">Number of bytes to copy from the old buffer into the new buffer.</param>
	extern "C" void ResizeToAtLeast(BufferPoolHandle bufferPoolHandle, ByteBuffer * buffer, int32_t targetSizeInBytes, int32_t copyCount);
	/// <summary>
	/// Returns a buffer to the buffer pool.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to return the buffer to.</param>
	/// <param name="buffer">Buffer to return to the pool.</param>
	extern "C" void Deallocate(BufferPoolHandle bufferPoolHandle, ByteBuffer * buffer);
	/// <summary>
	/// Returns a buffer to the buffer pool by its id.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to return the buffer to.</param>
	/// <param name="bufferId">Id of the buffer to return to the pool.</param>
	extern "C" void DeallocateById(BufferPoolHandle bufferPoolHandle, int32_t bufferId);
	/// <summary>
	/// Creates a new thread dispatcher.
	/// </summary>
	/// <param name="threadCount">Number of threads to use within the thread dispatcher.</param>
	/// <param name="threadPoolAllocationBlockSize">Minimum size in bytes of blocks allocated in per-thread buffer pools. Allocations requiring more space can result in larger block sizes, but no pools will allocate smaller blocks.</param>
	extern "C" ThreadDispatcherHandle CreateThreadDispatcher(int32_t threadCount, int32_t threadPoolAllocationBlockSize = 16384);
	/// <summary>
	/// Releases all resources held by a thread dispatcher and invalidates its handle.
	/// </summary>
	/// <param name="handle">Thread dispatcher to destroy.</param>
	extern "C" void DestroyThreadDispatcher(ThreadDispatcherHandle handle);
	/// <summary>
	/// Releases all resources held by a thread dispatcher and invalidates its handle.
	/// </summary>
	/// <param name="handle">Thread dispatcher to check the thread count of.</param>
	extern "C" int32_t GetThreadCount(ThreadDispatcherHandle handle);
	/// <summary>
	/// Creates a new simulation.
	/// </summary>
	/// <param name="bufferPool">Buffer pool for the simulation's main allocations.</param>
	/// <param name="narrowPhaseCallbacks">Narrow phase callbacks to be invoked by the simulation.</param>
	/// <param name="poseIntegratorCallbacks">Pose integration state and callbacks to be invoked by the simulation.</param>
	/// <param name="solveDescriptionInterop">Defines velocity iteration count and substep counts for the simulation's solver.</param>
	/// <param name="initialAllocationSizes">Initial capacities to allocate within the simulation.</param>
	/// <returns></returns>
	extern "C" SimulationHandle CreateSimulation(BufferPoolHandle bufferPool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacks poseIntegratorCallbacks, SolveDescription solveDescriptionInterop, SimulationAllocationSizes initialAllocationSizes);
	extern "C" void DestroySimulation(SimulationHandle handle);
	extern "C" BodyHandle AddBody(SimulationHandle simulationHandle, BodyDescription bodyDescription);
	extern "C" void RemoveBody(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	/// <summary>
	/// Gets a pointer to the dynamic state associated with a body. Includes pose, velocity, and inertia.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a body's state from.</param>
	/// <param name="bodyHandle">Body handle to pull data about.</param>
	/// <returns>Pointer to the body's dynamic state.</returns>
	/// <remarks>This is a direct pointer. The memory location associated with a body can move other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
	extern "C" BodyDynamics * GetBodyDynamics(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	/// <summary>
	/// Gets a pointer to the collidable associated with a body.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a body's state from.</param>
	/// <param name="bodyHandle">Body handle to pull data about.</param>
	/// <returns>Pointer to the body's collidable.</returns>
	/// <remarks>This is a direct pointer. The memory location associated with a body can move if other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
	extern "C" Collidable * GetBodyCollidable(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	/// <summary>
	/// Gets a pointer to the activity state associated with a body.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a body's state from.</param>
	/// <param name="bodyHandle">Body handle to pull data about.</param>
	/// <returns>Pointer to the body's activity state.</returns>
	/// <remarks>This is a direct pointer. The memory location associated with a body can move if other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
	extern "C" BodyActivity * GetBodyActivity(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	/// <summary>
	/// Gets a pointer to the list of constraints associated with a body.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a body's state from.</param>
	/// <param name="bodyHandle">Body handle to pull data about.</param>
	/// <returns>Pointer to the body's constraint list.</returns>
	/// <remarks>This is a direct pointer. The memory location associated with a body can move if other bodies are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
	extern "C" QuickList<BodyConstraintReference>*GetBodyConstraints(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	/// <summary>
	/// Gets a description of a body.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a body's state from.</param>
	/// <param name="bodyHandle">Body handle to pull data about.</param>
	/// <returns>Description of a body.</returns>
	extern "C" BodyDescription GetBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	/// <summary>
	/// Applies a description to a body.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a body's state from.</param>
	/// <param name="bodyHandle">Body handle to pull data about.</param>
	/// <param name="description">Description to apply to the body.</param>
	extern "C" void ApplyBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle, BodyDescription description);
	extern "C" StaticHandle AddStatic(SimulationHandle simulationHandle, StaticDescription staticDescription);
	extern "C" void RemoveStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	/// <summary>
	/// Gets a pointer to data associated with a static.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a static's state from.</param>
	/// <param name="staticHandle">Static handle to pull data about.</param>
	/// <returns>Pointer to the static's data.</returns>
	/// <remarks>This is a direct pointer. The memory location associated with a static can move if other statics are removed from the simulation; do not hold a pointer beyond the point where it may be invalidated.</remarks>
	extern "C" Static * GetStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	/// <summary>
	/// Gets a static's description.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a static's state from.</param>
	/// <param name="staticHandle">Static handle to pull data about.</param>
	/// <returns>Description of the static..</returns>
	extern "C" StaticDescription GetStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle);
	/// <summary>
	/// Applies a description to a static.
	/// </summary>
	/// <param name="simulationHandle">Simulation to pull a static's state from.</param>
	/// <param name="staticHandle">Static handle to pull data about.</param>
	extern "C" void ApplyStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle, StaticDescription description);
	/// <summary>
	/// Steps the simulation forward a single time.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to step.</param>
	/// <param name="dt">Duration of the timestep.</param>
	/// <param name="threadDispatcherHandle">Handle of the thread dispatcher to use, if any. Can be a null reference.</param>
	extern "C" void Timestep(SimulationHandle simulationHandle, float dt, ThreadDispatcherHandle threadDispatcherHandle);
	/// <summary>
	/// Grabs a collidable's bounding boxes in the broad phase.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
	/// <param name="bodyHandle">Body to pull bounding box data about.</param>
	/// <param name="min">Minimum bounds of the collidable's bounding box.</param>
	/// <param name="max">Maximum bounds of the collidable's bounding box.</param>
	extern "C" void GetBodyBoundingBoxInBroadPhase(SimulationHandle simulationHandle, BodyHandle bodyHandle, Vector3 * min, Vector3 * max);
	/// <summary>
	/// Grabs a collidable's bounding boxes in the broad phase.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
	/// <param name="staticHandle">Static to pull bounding box data about.</param>
	/// <param name="min">Minimum bounds of the collidable's bounding box.</param>
	/// <param name="max">Maximum bounds of the collidable's bounding box.</param>
	extern "C" void GetStaticBoundingBoxInBroadPhase(SimulationHandle simulationHandle, StaticHandle staticHandle, Vector3 * min, Vector3 * max);
	/// <summary>
	/// Gets the mapping from body handles to the body's location in storage.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
	/// <param name="bodyHandleToIndexMapping">Mapping from a body handle to the body's memory location.</param>
	/// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it.</remarks>
	extern "C" void GetBodyHandleToLocationMapping(SimulationHandle simulationHandle, Buffer<BodyMemoryLocation>*bodyHandleToIndexMapping);
	/// <summary>
	/// Gets the body sets for a simulation. Slot 0 is the active set. Subsequent sets are sleeping. Not every slot beyond slot 0 is filled.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
	/// <param name="bodySets">Mapping from a body handle to the body's memory location.</param>
	/// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it.</remarks>
	extern "C" void GetBodySets(SimulationHandle simulationHandle, Buffer<BodySet>*bodySets);
	/// <summary>
	/// Gets the mapping from body handles to the body's location in storage.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
	/// <param name="staticHandleToIndexMapping">Mapping from a static handle to the static's memory location.</param>
	/// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it.</remarks>
	extern "C" void GetStaticHandleToLocationMapping(SimulationHandle simulationHandle, Buffer<int32_t>*staticHandleToIndexMapping);
	/// <summary>
	/// Gets the statics set for a simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to pull data from.</param>
	/// <param name="statics">The set of all statics within a simulation.</param>
	/// <param name="count">Number of statics in the simulation.</param>
	/// <remarks>The buffer returned by this function can be invalidated if the simulation resizes it. The count is a snapshot.</remarks>
	extern "C" void GetStatics(SimulationHandle simulationHandle, Buffer<Static>*statics, int32_t * count);
	/// <summary>
	/// Adds a sphere shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="sphere">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddSphere(SimulationHandle simulationHandle, Sphere sphere);
	/// <summary>
	/// Adds a capsule shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="capsule">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddCapsule(SimulationHandle simulationHandle, Capsule capsule);
	/// <summary>
	/// Adds a box shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="box">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddBox(SimulationHandle simulationHandle, Box box);
	/// <summary>
	/// Adds a triangle shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="triangle">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddTriangle(SimulationHandle simulationHandle, Triangle triangle);
	/// <summary>
	/// Adds a cylinder shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="cylinder">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddCylinder(SimulationHandle simulationHandle, Cylinder cylinder);
	/// <summary>
	/// Adds a convex hull shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="convexHull">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddConvexHull(SimulationHandle simulationHandle, ConvexHull convexHull);
	/// <summary>
	/// Adds a compound shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="bigCompound">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddCompound(SimulationHandle simulationHandle, Compound bigCompound);
	/// <summary>
	/// Adds a big compound shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="bigCompound">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddBigCompound(SimulationHandle simulationHandle, BigCompound bigCompound);
	/// <summary>
	/// Adds a mesh shape to the simulation.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to add the shape to.</param>
	/// <param name="mesh">Shape to add to the simulation.</param>
	extern "C" TypedIndex AddMesh(SimulationHandle simulationHandle, Mesh mesh);
	/// <summary>
	/// Removes a shape from the simulation. Does not return any shape allocated buffers to buffer pools.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape to remove from the simulation.</param>
	extern "C" void RemoveShape(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Removes a shape from the simulation. If the shape has resources that were allocated from a buffer pool, they will be returned to the specified pool.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="bufferPoolHandle">Buffer pool to return shape resources to, if any.</param>
	/// <param name="shape">Shape to remove from the simulation.</param>
	/// <remarks>The same buffer pool must be used for both allocation and deallocation.</remarks>
	extern "C" void RemoveAndDestroyShape(SimulationHandle simulationHandle, BufferPoolHandle bufferPoolHandle, TypedIndex shape);
	/// <summary>
	/// Removes a shape and all references child shapes from the simulation. If the shapes had resources that were allocated from a buffer pool, they will be returned to the specified pool.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="bufferPoolHandle">Buffer pool to return shape resources to, if any.</param>
	/// <param name="shape">Shape to remove from the simulation.</param>
	/// <remarks>The same buffer pool must be used for both allocation and deallocation.</remarks>
	extern "C" void RemoveAndDestroyShapeRecursively(SimulationHandle simulationHandle, BufferPoolHandle bufferPoolHandle, TypedIndex shape);
	/// <summary>
	/// Creates a convex hull shape from a point set.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to allocate resources from for the compound's acceleration structures.</param>
	/// <param name="points">Points in the convex hull.</param>
	/// <param name="centerOfMass">Center of mass computed for the hull and subtracted from all the points in the points used for the final shape.</param>
	extern "C" ConvexHull CreateConvexHull(BufferPoolHandle bufferPoolHandle, Buffer<CompoundChild> points, Vector3 * centerOfMass);
	/// <summary>
	/// Returns buffers allocated for a convex hull shape.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
	/// <param name="convexHull">Convex hull to destroy.</param>
	extern "C" void DestroyConvexHull(BufferPoolHandle bufferPoolHandle, ConvexHull * convexHull);
	/// <summary>
	/// Returns buffers allocated for a compound shape.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
	/// <param name="compound">Compound to destroy.</param>
	extern "C" void DestroyCompound(BufferPoolHandle bufferPoolHandle, Compound * compound);
	/// <summary>
	/// Creates a big compound shape from a list of children.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to which the shapes referenced by the compound children belong.</param>
	/// <param name="bufferPoolHandle">Buffer pool to allocate resources from for the compound's acceleration structures.</param>
	/// <param name="children">Children of the compound.</param>
	extern "C" BigCompound CreateBigCompound(SimulationHandle simulationHandle, BufferPoolHandle bufferPoolHandle, Buffer<CompoundChild> children);
	/// <summary>
	/// Returns buffers allocated for a big compound shape.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
	/// <param name="bigCompound">Big compound to destroy.</param>
	extern "C" void DestroyBigCompound(BufferPoolHandle bufferPoolHandle, BigCompound * bigCompound);
	/// <summary>
	/// Creates a mesh shape from triangles.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to allocate resources from for the compound's acceleration structures.</param>
	/// <param name="triangles">Triangles composing the mesh.</param>
	/// <param name="scale">Scale of the mesh.</param>
	/// <remarks>This uses a pretty old sweep builder. Large meshes will take a while. There are ways to do this much faster if required; see https://github.com/bepu/bepuphysics2/blob/master/Demos/DemoMeshHelper.cs#L186.</remarks>
	extern "C" Mesh CreateMesh(BufferPoolHandle bufferPoolHandle, Buffer<Triangle> triangles, Vector3 scale);
	/// <summary>
	/// Returns buffers allocated for a mesh shape.
	/// </summary>
	/// <param name="bufferPoolHandle">Buffer pool to return resources to. Must be the same pool that resources were allocated from.</param>
	/// <param name="mesh">Mesh to destroy.</param>
	extern "C" void DestroyMesh(BufferPoolHandle bufferPoolHandle, Mesh * mesh);
	/// <summary>
	/// Computes the inertia of a sphere.
	/// </summary>
	/// <param name="sphere">Shape to compute the inertia of.</param>
	/// <param name="mass">Mass to use in the inertia calculation.</param>
	/// <returns>Inertia of the shape.</returns>
	extern "C" BodyInertia ComputeSphereInertia(Sphere sphere, float mass);
	/// <summary>
	/// Computes the inertia of a capsule.
	/// </summary>
	/// <param name="capsule">Shape to compute the inertia of.</param>
	/// <param name="mass">Mass to use in the inertia calculation.</param>
	/// <returns>Inertia of the shape.</returns>
	extern "C" BodyInertia ComputeCapsuleInertia(Capsule capsule, float mass);
	/// <summary>
	/// Computes the inertia of a box.
	/// </summary>
	/// <param name="box">Shape to compute the inertia of.</param>
	/// <param name="mass">Mass to use in the inertia calculation.</param>
	/// <returns>Inertia of the shape.</returns>
	extern "C" BodyInertia ComputeBoxInertia(Box box, float mass);
	/// <summary>
	/// Computes the inertia of a triangle.
	/// </summary>
	/// <param name="triangle">Shape to compute the inertia of.</param>
	/// <param name="mass">Mass to use in the inertia calculation.</param>
	/// <returns>Inertia of the shape.</returns>
	extern "C" BodyInertia ComputeTriangleInertia(Triangle triangle, float mass);
	/// <summary>
	/// Computes the inertia of a cylinder.
	/// </summary>
	/// <param name="cylinder">Shape to compute the inertia of.</param>
	/// <param name="mass">Mass to use in the inertia calculation.</param>
	/// <returns>Inertia of the shape.</returns>
	extern "C" BodyInertia ComputeCylinderInertia(Cylinder cylinder, float mass);
	/// <summary>
	/// Computes the inertia of a convex hull.
	/// </summary>
	/// <param name="convexHull">Shape to compute the inertia of.</param>
	/// <param name="mass">Mass to use in the inertia calculation.</param>
	/// <returns>Inertia of the shape.</returns>
	extern "C" BodyInertia ComputeConvexHullInertia(ConvexHull convexHull, float mass);
	/// <summary>
	/// Computes the inertia of a convex.
	/// </summary>
	/// <param name="convex">Index of a convex to calculate the inertia for.</param>
	/// <param name="mass">Mass to use in the inertia calculation.</param>
	/// <returns>Inertia of the shape. If the shape index was not a convex, this returns a zeroed inverse inertia tensor.</returns>
	extern "C" BodyInertia ComputeConvexInertia(SimulationHandle simulationHandle, TypedIndex convex, float mass);
	/// <summary>
	/// Computes the inertia associated with a set of compound children. Does not recenter the children.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to which the shapes referenced by the compound children belong.</param>
	/// <param name="children">Children of the compound.</param>
	/// <param name="childMasses">Masses of the children composing the compound.</param>
	extern "C" BodyInertia ComputeCompoundInertia(SimulationHandle simulationHandle, Buffer<CompoundChild> children, Buffer<float> childMasses);
	/// <summary>
	/// Computes the inertia associated with a set of compound children. Recenters all children onto the computed local center of mass.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to which the shapes referenced by the compound children belong.</param>
	/// <param name="children">Children of the compound.</param>
	/// <param name="childMasses">Masses of the children composing the compound.</param>
	/// <param name="centerOfMass">Computed center of mass that was subtracted from the position of compound children.</param>
	extern "C" BodyInertia ComputeCompoundInertiaWithRecentering(SimulationHandle simulationHandle, Buffer<CompoundChild> children, Buffer<float> childMasses, Vector3 * centerOfMass);
	/// <summary>
	/// Computes the inertia associated with a mesh by treating its triangles as a soup with no volume. Does not recenter the triangles on a computed center of mass.
	/// </summary>
	/// <param name="mesh">Mesh to compute the inertia of.</param>
	/// <param name="mass">Mass of the mesh.</param>
	extern "C" BodyInertia ComputeOpenMeshInertia(Mesh mesh, float mass);
	/// <summary>
	/// Computes the inertia associated with a mesh by treating it as a closed volume. Does not recenter the triangles on a computed center of mass.
	/// </summary>
	/// <param name="mesh">Mesh to compute the inertia of.</param>
	/// <param name="mass">Mass of the mesh.</param>
	extern "C" BodyInertia ComputeClosedMeshInertia(Mesh mesh, float mass);
	/// <summary>
	/// Computes the inertia associated with a mesh by treating its triangles as a soup with no volume. Recenters all children onto the computed local center of mass.
	/// </summary>
	/// <param name="mesh">Mesh to compute the inertia of.</param>
	/// <param name="mass">Mass of the mesh.</param>
	extern "C" BodyInertia ComputeOpenMeshInertiaWithRecentering(Mesh mesh, float mass, Vector3 * centerOfMass);
	/// <summary>
	/// Computes the inertia associated with a mesh by treating it as a closed volume. Recenters all children onto the computed local center of mass.
	/// </summary>
	/// <param name="mesh">Mesh to compute the inertia of.</param>
	/// <param name="mass">Mass of the mesh.</param>
	extern "C" BodyInertia ComputeClosedMeshInertiaWithRecentering(Mesh mesh, float mass, Vector3 * centerOfMass);
	/// <summary>
	/// Gets a pointer to a sphere shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" Sphere * GetSphereShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Gets a pointer to a capsule shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" Capsule * GetCapsuleShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Gets a pointer to a box shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" Box * GetBoxShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Gets a pointer to a triangle shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" Triangle * GetTriangleShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Gets a pointer to a cylinder shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" Cylinder * GetCylinderShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Gets a pointer to a convex hull shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" ConvexHull * GetConvexHullShapeData(SimulationHandle simulationHandle, TypedIndex shape);
	/// <summary>
	/// Gets a pointer to a compound shape's data stored within the simulation's shapes buffers.
	/// </summary>
	/// <param name="simulationHandle">Handle of the simulation to remove the shape from.</param>
	/// <param name="shape">Shape reference to request from the simulation.</param>
	/// <returns>Pointer to the shape's data in the simulation's shapes buffers.</returns>
	extern "C" Compound * GetCompoundShapeData(SimulationHandle simulationHandle, TypedIndex shape);

}