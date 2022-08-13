#include "BepuPhysics.h"
#include <cmath> 
#include <iostream>
#include <string>
using namespace Bepu;

//NARROW PHASE 
struct NarrowPhaseSettings
{
	PairMaterialProperties MaterialProperties;
};

//If you had multiple simulations, you could index settings by the simulationHandle.Index. //TODO: That's not exposed yet!
NarrowPhaseSettings narrowPhaseSettings;

bool AllowContactGeneration(SimulationHandle simulationHandle, int32_t workerIndex, CollidableReference a, CollidableReference b, float* speculativeMargin)
{
	//While the engine won't even try creating pairs between statics at all, it will ask about kinematic-kinematic pairs.
	//Those pairs cannot emit constraints since both involved bodies have infinite inertia. Since most of the demos don't need
	//to collect information about kinematic-kinematic pairs, we'll require that at least one of the bodies needs to be dynamic.
	return a.GetMobility() == CollidableMobility::Dynamic || b.GetMobility() == CollidableMobility::Dynamic;
}

bool AllowContactGenerationBetweenChildren(SimulationHandle simulationHandle, int32_t workerIndex, CollidablePair collidablePair, int32_t childIndexA, int32_t childIndexB)
{
	return true;
}

//On the C# side, these two functions are one generic function, but it got split up due to the interop barrier.
bool ConfigureConvexContactManifold(SimulationHandle, int32_t workerIndex, CollidablePair collidablePair, ConvexContactManifold* contactManifold, PairMaterialProperties* materialProperties)
{
	*materialProperties = narrowPhaseSettings.MaterialProperties;
	return true;
}

bool ConfigureNonconvexContactManifold(SimulationHandle simulationHandle, int32_t workerIndex, CollidablePair collidablePair, NonconvexContactManifold* contactManifold, PairMaterialProperties* materialProperties)
{
	*materialProperties = narrowPhaseSettings.MaterialProperties;
	return true;
}

bool ConfigureChildContactManifold(SimulationHandle simulationHandle, int32_t workerIndex, CollidablePair collidablePair, int32_t childIndexA, int32_t childIndexB, ConvexContactManifold* contactManifold)
{
	return true;
}

//POSE INTEGRATION
struct PoseIntegrationSettings
{
	Vector3 Gravity;
	float LinearDamping;
	float AngularDamping;

	Vector3 GravityDt;
	float LinearDampingDt;
	float AngularDampingDt;
};

PoseIntegrationSettings poseIntegrationSettings;

void PrepareForIntegration(SimulationHandle simulation, float dt)
{
	//No reason to recalculate gravity * dt for every body; just cache it ahead of time.
	//Since these callbacks don't use per-body damping values, we can precalculate everything.
	poseIntegrationSettings.LinearDampingDt = std::powf(std::fmin(std::fmax(1 - poseIntegrationSettings.LinearDamping, 0), 1), dt);
	poseIntegrationSettings.AngularDampingDt = std::powf(std::fmin(std::fmax(1 - poseIntegrationSettings.AngularDamping, 0), 1), dt);
	//TODO: Interop has no simd fanciness or operators or anything! Would be nice to replace.
	poseIntegrationSettings.GravityDt.X = poseIntegrationSettings.Gravity.X * dt;
	poseIntegrationSettings.GravityDt.Y = poseIntegrationSettings.Gravity.Y * dt;
	poseIntegrationSettings.GravityDt.Z = poseIntegrationSettings.Gravity.Z * dt;
}

void IntegrateVelocityScalar(SimulationHandle simulation, int32_t bodyIndex, Vector3 position, Quaternion orientation, BodyInertia localInertia, int32_t workerIndex, float dt, BodyVelocity* velocity)
{
	//TODO: Interop has no simd fanciness or operators or anything! Would be nice to replace. Particularly if we could use AOSOA types here!
	velocity->Linear.X = (velocity->Linear.X + poseIntegrationSettings.GravityDt.X) * poseIntegrationSettings.LinearDampingDt;
	velocity->Linear.Y = (velocity->Linear.Y + poseIntegrationSettings.GravityDt.Y) * poseIntegrationSettings.LinearDampingDt;
	velocity->Linear.Z = (velocity->Linear.Z + poseIntegrationSettings.GravityDt.Z) * poseIntegrationSettings.LinearDampingDt;
	velocity->Angular.X = velocity->Angular.X * poseIntegrationSettings.LinearDampingDt;
	velocity->Angular.Y = velocity->Angular.Y * poseIntegrationSettings.LinearDampingDt;
	velocity->Angular.Z = velocity->Angular.Z * poseIntegrationSettings.LinearDampingDt;
}

#include "CollidableProperty.h"
CollidableProperty<int32_t> ints;

int main()
{
	Initialize();

	BufferPoolHandle pool = CreateBufferPool();
	int32_t threadCount = GetPlatformThreadCount();
	threadCount = threadCount > 4 ? threadCount - 2 : threadCount;
	ThreadDispatcherHandle threadDispatcher = CreateThreadDispatcher(threadCount);

	NarrowPhaseCallbacks narrowPhaseCallbacks = {};
	narrowPhaseCallbacks.AllowContactGenerationFunction = &AllowContactGeneration;
	narrowPhaseCallbacks.AllowContactGenerationBetweenChildrenFunction = &AllowContactGenerationBetweenChildren;
	narrowPhaseCallbacks.ConfigureConvexContactManifoldFunction = &ConfigureConvexContactManifold;
	narrowPhaseCallbacks.ConfigureNonconvexContactManifoldFunction = &ConfigureNonconvexContactManifold;
	narrowPhaseCallbacks.ConfigureChildContactManifoldFunction = &ConfigureChildContactManifold;

	narrowPhaseSettings.MaterialProperties = PairMaterialProperties(1, 2, SpringSettings(30, 1));

	PoseIntegratorCallbacks poseIntegratorCallbacks = {};
	poseIntegratorCallbacks.AngularIntegrationMode = AngularIntegrationMode::Nonconserving;
	poseIntegratorCallbacks.AllowSubstepsForUnconstrainedBodies = false;
	poseIntegratorCallbacks.IntegrateVelocityForKinematics = false;
	//While on the C# side velocity integration is exposed with inlined callbacks that operate on AoSoA vector bundles of bodies, 
	//vectorizing things properly on the native side would be a pain so we'll ask the callback to transpose everything into simple AoS.
	//It's a performance hit, but that's fine for now.
	poseIntegratorCallbacks.UseScalarCallback = true;
	poseIntegratorCallbacks.PrepareForIntegration = &PrepareForIntegration;
	poseIntegratorCallbacks.IntegrateVelocityScalar = &IntegrateVelocityScalar;
	poseIntegrationSettings = { Vector3 { 0, -10.0f, 0 }, 0.01f, 0.01f };

	SimulationHandle simulation = CreateSimulation(pool, narrowPhaseCallbacks, poseIntegratorCallbacks, SolveDescription(4, 1), SimulationAllocationSizes());

	//Create a floor to drop stuff on!
	AddStatic(simulation, StaticDescription::Create(Vector3(), Quaternion::GetIdentity(), AddBox(simulation, Box(100, 1, 100))));

	//Drop some boxes on it!
	BodyInertia inertia = { Symmetric3x3 { 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f}, 1.0f };
	BodyDescription bodyDescription = BodyDescription::CreateDynamic(RigidPose(Vector3()), inertia, CollidableDescription(AddBox(simulation, Box(1, 1, 1))), BodyActivityDescription(0.01f));

	const int bodyCount = 100;
	BodyHandle bodyHandles[bodyCount];
	for (int i = 0; i < bodyCount; ++i)
	{
		bodyDescription.Pose.Position.Y = 1.0f + i * 1.5f;
		bodyHandles[i] = AddBody(simulation, bodyDescription);
	}

	for (int i = 0; i < 1000; ++i)
	{
		Timestep(simulation, 1.0f / 60.0f, InstanceHandle());
		BodyDynamics* dynamics = GetBodyDynamics(simulation, bodyHandles[bodyCount - 1]);
		std::cout << dynamics->Motion.Pose.Position.Y << "\n";
	}

	Destroy();
}