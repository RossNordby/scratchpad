#pragma once
#include <stdint.h>

typedef int32_t BodyHandle;
typedef int32_t StaticHandle;
typedef int32_t SimulationHandle;
typedef int32_t BufferPoolHandle;
typedef int32_t ThreadDispatcherHandle;

namespace Bepu
{
	extern "C" void Initialize();
	extern "C" void Destroy();
	extern "C" SIMDWidth GetSIMDWidth();
	extern "C" BufferPoolHandle CreateBufferPool(int32_t minimumBlockAllocationSize, int32_t expectedUsedSlotCountPerPool);
	extern "C" void ClearBufferPool(BufferPoolHandle handle);
	extern "C" void DestroyBufferPool(BufferPoolHandle handle);
	extern "C" ThreadDispatcherHandle CreateThreadDispatcher(int32_t threadCount, int32_t threadPoolAllocationBlockSize);
	extern "C" void DestroyThreadDispatcher(ThreadDispatcherHandle handle);
	extern "C" int32_t GetThreadCount(ThreadDispatcherHandle handle);
	extern "C" SimulationHandle CreateSimulation(BufferPoolHandle bufferPool, NarrowPhaseCallbacksInterop narrowPhaseCallbacksInterop, PoseIntegratorCallbacksInterop poseIntegratorCallbacksInterop, SolveDescriptionInterop solveDescriptionInterop, SimulationAllocationSizes initialAllocationSizes);
	extern "C" void DestroySimulation(SimulationHandle handle);
	extern "C" BodyHandle AddBody(SimulationHandle simulationHandle, BodyDescription bodyDescription);
	extern "C" void RemoveBody(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyDynamics * GetBodyDynamics(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" Collidable * GetBodyCollidable(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyActivity * GetBodyActivity(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" QuickList`1* GetBodyConstraints(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyDescription GetBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" void ApplyBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle, BodyDescription description);
	extern "C" StaticHandle AddStatic(SimulationHandle simulationHandle, StaticDescription staticDescription);
	extern "C" void RemoveStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" Static * GetStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" StaticDescription GetStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" void ApplyStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle, StaticDescription description);
	extern "C" void Timestep(SimulationHandle simulationHandle, float dt, ThreadDispatcherHandle threadDispatcherHandle);

}