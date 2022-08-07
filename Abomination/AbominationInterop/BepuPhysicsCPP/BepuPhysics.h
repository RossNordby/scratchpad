#pragma once

#include <stdint.h>
#include <assert.h>
#include "Math.h"
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

	extern "C" void Initialize();
	extern "C" void Destroy();
	extern "C" SIMDWidth GetSIMDWidth();
	extern "C" int32_t GetPlatformThreadCount();
	extern "C" BufferPoolHandle CreateBufferPool(int32_t minimumBlockAllocationSize = 131072, int32_t expectedUsedSlotCountPerPool = 16);
	extern "C" void ClearBufferPool(BufferPoolHandle handle);
	extern "C" void DestroyBufferPool(BufferPoolHandle handle);
	extern "C" ThreadDispatcherHandle CreateThreadDispatcher(int32_t threadCount, int32_t threadPoolAllocationBlockSize = 16384);
	extern "C" void DestroyThreadDispatcher(ThreadDispatcherHandle handle);
	extern "C" int32_t GetThreadCount(ThreadDispatcherHandle handle);
	extern "C" SimulationHandle CreateSimulation(BufferPoolHandle bufferPool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacks poseIntegratorCallbacks, SolveDescription solveDescriptionInterop, SimulationAllocationSizes initialAllocationSizes);
	extern "C" void DestroySimulation(SimulationHandle handle);
	extern "C" BodyHandle AddBody(SimulationHandle simulationHandle, BodyDescription bodyDescription);
	extern "C" void RemoveBody(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyDynamics * GetBodyDynamics(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" Collidable * GetBodyCollidable(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyActivity * GetBodyActivity(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" QuickList<BodyConstraintReference>*GetBodyConstraints(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyDescription GetBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" void ApplyBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle, BodyDescription description);
	extern "C" StaticHandle AddStatic(SimulationHandle simulationHandle, StaticDescription staticDescription);
	extern "C" void RemoveStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" Static * GetStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" StaticDescription GetStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" void ApplyStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle, StaticDescription description);
	extern "C" void Timestep(SimulationHandle simulationHandle, float dt, ThreadDispatcherHandle threadDispatcherHandle);
	extern "C" TypedIndex AddSphere(SimulationHandle simulationHandle, Sphere sphere);
	extern "C" TypedIndex AddCapsule(SimulationHandle simulationHandle, Capsule capsule);
	extern "C" TypedIndex AddBox(SimulationHandle simulationHandle, Box box);
	extern "C" TypedIndex AddTriangle(SimulationHandle simulationHandle, Triangle triangle);
	extern "C" TypedIndex AddCylinder(SimulationHandle simulationHandle, Cylinder cylinder);
	extern "C" TypedIndex AddConvexHull(SimulationHandle simulationHandle, ConvexHull convexHull);
	extern "C" TypedIndex AddCompound(SimulationHandle simulationHandle, Compound bigCompound);
	extern "C" TypedIndex AddBigCompound(SimulationHandle simulationHandle, BigCompound bigCompound);
	extern "C" TypedIndex AddMesh(SimulationHandle simulationHandle, Mesh mesh);
	extern "C" void RemoveShape(SimulationHandle simulationHandle, TypedIndex shape);
	extern "C" void RemoveAndDestroyShape(SimulationHandle simulationHandle, BufferPoolHandle bufferPoolHandle, TypedIndex shape);
	extern "C" void RemoveAndDestroyShapeRecursively(SimulationHandle simulationHandle, BufferPoolHandle bufferPoolHandle, TypedIndex shape);
	extern "C" BigCompound CreateBigCompound(SimulationHandle simulationHandle, BufferPoolHandle bufferPoolHandle, Buffer<CompoundChild> children);
	extern "C" void DestroyBigCompound(BufferPoolHandle bufferPoolHandle, BigCompound * bigCompound);
	extern "C" BodyInertia ComputeCompoundInertiaWithoutRecentering(SimulationHandle simulationHandle, Buffer<CompoundChild> children, Buffer<float> childMasses);
	extern "C" BodyInertia ComputeCompoundInertia(SimulationHandle simulationHandle, Buffer<CompoundChild> children, Buffer<float> childMasses, Vector3 * centerOfMass);

}