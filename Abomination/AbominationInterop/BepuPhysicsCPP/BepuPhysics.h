#pragma once
#include <stdint.h>
#include <assert.h>
#include <limits>

namespace Bepu
{
	typedef int32_t BodyHandle;
	typedef int32_t StaticHandle;
	typedef int32_t ConstraintHandle;
	typedef int32_t SimulationHandle;
	typedef int32_t BufferPoolHandle;
	typedef int32_t ThreadDispatcherHandle;
	enum SIMDWidth { SIMD128, SIMD256 };

	struct Vector3
	{
		float X;
		float Y;
		float Z;
	};
	struct Quaternion
	{
		float X;
		float Y;
		float Z;
		float W;
	};

	/// <summary>
	/// Linear and angular velocity for a body.
	/// </summary>
	struct BodyVelocity
	{
		/// <summary>
		/// Linear velocity associated with the body.
		/// </summary>
		Vector3 Linear;
		int32_t Pad0;
		/// <summary>
		/// Angular velocity associated with the body.
		/// </summary>
		Vector3 Angular;
		int32_t Pad1;
	};

	/// <summary>
	/// Represents a rigid transformation.
	/// </summary>
	struct RigidPose
	{
		/// <summary>
		/// Orientation of the pose.
		/// </summary>
		Quaternion Orientation;
		/// <summary>
		/// Position of the pose.
		/// </summary>
		Vector3 Position;
		int32_t Pad;
	};

	/// <summary>
	/// Describes the pose and velocity of a body.
	/// </summary>
	struct MotionState
	{
		/// <summary>
		/// Pose of the body.
		/// </summary>
		RigidPose Pose;
		/// <summary>
		/// Linear and angular velocity of the body.
		/// </summary>
		BodyVelocity Velocity;
	};

	/// <summary>
	/// Lower left triangle (including diagonal) of a symmetric 3x3 matrix.
	/// </summary>
	struct alignas(float) Symmetric3x3
	{
		/// <summary>
		/// First row, first column of the matrix.
		/// </summary>
		float XX;
		/// <summary>
		/// Second row, first column of the matrix.
		/// </summary>
		float YX;
		/// <summary>
		/// Second row, second column of the matrix.
		/// </summary>
		float YY;
		/// <summary>
		/// Third row, first column of the matrix.
		/// </summary>
		float ZX;
		/// <summary>
		/// Third row, second column of the matrix.
		/// </summary>
		float ZY;
		/// <summary>
		/// Third row, third column of the matrix.
		/// </summary>
		float ZZ;
	};

	/// <summary>
	/// Stores the inertia for a body.
	/// </summary>
	/// <remarks>This representation stores the inverse mass and inverse inertia tensor. Most of the high frequency use cases in the engine naturally use the inverse.</remarks>

	struct BodyInertia
	{
		/// <summary>
		/// Inverse of the body's inertia tensor.
		/// </summary>
		Symmetric3x3 InverseInertiaTensor;
		/// <summary>
		/// Inverse of the body's mass.
		/// </summary>
		float InverseMass;
		uint32_t Pad;
	};

	/// <summary>
	/// Stores the local and world views of a body's inertia, packed together for efficient access.
	/// </summary>
	struct BodyInertias
	{
		/// <summary>
		/// Local inertia of the body.
		/// </summary>
		BodyInertia Local;
		/// <summary>
		/// Transformed world inertia of the body. Note that this is only valid between the velocity integration that updates it and the pose integration that follows.
		/// Outside of that execution window, this should be considered undefined.
		/// </summary>
		/// <remarks>
		/// We cache this here because velocity integration wants both the local and world inertias, and any integration happening within the solver will do so without the benefit of sequential loads.
		/// In that context, being able to load a single cache line to grab both local and world inertia helps quite a lot.</remarks>
		BodyInertia World;
	};

	/// <summary>
	/// Stores all body information needed by the solver together.
	/// </summary>
	/// <remarks>
	struct BodyDynamics
	{
		/// <summary>
		/// Pose and velocity information for the body.
		/// </summary>
		MotionState Motion;
		/// <summary>
		/// Inertia information for the body.
		/// </summary>
		BodyInertias Inertia;
	};

	/// <summary>
	/// Represents an index with an associated type packed into a single integer.
	/// </summary>
	struct TypedIndex
	{
		/// <summary>
		/// Bit packed representation of the typed index.
		/// </summary>
		uint32_t Packed;

		/// <summary>
		/// Gets the type index of the object.
		/// </summary>
		int32_t GetType()
		{
			return (int32_t)(Packed & 0x7F000000) >> 24;
		}

		/// <summary>
		/// Gets the index of the object.
		/// </summary>
		int32_t GetIndex()
		{
			return (int32_t)(Packed & 0x00FFFFFF);
		}

		/// <summary>
		/// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
		/// </summary>
		bool Exists()
		{
			return (Packed & (1 << 31)) > 0;
		}
	};

	/// <summary>
	/// Defines how a collidable will handle collision detection in the presence of velocity.
	/// </summary>
	enum ContinuousDetectionMode : uint32_t
	{
		/// <summary>
		/// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
		/// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
		/// <para>This is the cheapest mode. If a Discrete mode collidable is moving quickly and the maximum speculative margin is limited,
		/// the fact that its bounding box is not expanded may cause it to miss a collision even with a non-Discrete collidable.</para>
		/// </summary>
		Discrete = 0,
		/// <summary>
		/// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
		/// <para>The collidable's bounding box will be expanded by velocity without being limited by the speculative margin.</para>
		/// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes 
		/// that should avoid missing collisions.</para>
		/// </summary>
		Passive = 1,
		/// <summary>
		/// <para>Collision detection will start with a sweep test to identify a likely time of impact. Speculative contacts will be generated for the predicted collision.</para>
		/// <para>This mode can capture angular motion with very few ghost collisions. It can, however, miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
		/// </summary>
		Continuous = 2,
	};

	/// <summary>
	/// Defines how a collidable handles collisions with significant velocity.
	/// </summary>
	struct ContinuousDetection
	{
		/// <summary>
		/// The continuous collision detection mode.
		/// </summary>
		ContinuousDetectionMode Mode;

		/// <summary>
		/// If using <see cref="ContinuousDetectionMode.Continuous"/>, this defines the minimum progress that the sweep test will make when searching for the first time of impact.
		/// Collisions lasting less than <see cref="MinimumSweepTimestep"/> may be missed by the sweep test. Using larger values can significantly increase the performance of sweep tests.
		/// </summary>
		float MinimumSweepTimestep;

		/// <summary>
		/// If using <see cref="ContinuousDetectionMode.Continuous"/>, sweep tests will terminate if the time of impact region has been refined to be smaller than <see cref="SweepConvergenceThreshold"/>.
		/// Values closer to zero will converge more closely to the true time of impact, but for speculative contact generation larger values usually work fine.
		/// Larger values allow the sweep to terminate much earlier and can significantly improve sweep performance.
		/// </summary>
		float SweepConvergenceThreshold;

		/// <summary>
		/// Gets whether the continuous collision detection configuration will permit bounding box expansion beyond the calculated speculative margin.
		/// </summary>
		bool AllowExpansionBeyondSpeculativeMargin() { return Mode > 0; }

		/// <summary>
		/// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
		/// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
		/// <para>This can be marginally cheaper than Passive modes if using a limited maximum speculative margin. If a Discrete mode collidable is moving quickly and the maximum speculative margin is limited,
		/// the fact that its bounding box is not expanded may cause it to miss a collision even with a non-Discrete collidable.</para>
		/// <para>Note that Discrete and Passive only differ if maximum speculative margin is restricted.</para>
		/// </summary>
		/// <returns>Detection settings for the given discrete configuration.</returns>
		static ContinuousDetection Discrete()
		{
			ContinuousDetection detection;
			detection.Mode = ContinuousDetectionMode::Discrete;
			detection.MinimumSweepTimestep = 0;
			detection.SweepConvergenceThreshold = 0;
			return detection;
		}

		/// <summary>
		/// <para>No sweep tests are performed. Default speculative contact generation will occur within the speculative margin.</para>
		/// <para>The collidable's bounding box and speculative margin will be expanded by velocity.</para>
		/// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes that should avoid missing collisions.</para>
		/// </summary>
		/// <returns>Detection settings for the passive configuration.</returns>
		static ContinuousDetection Passive()
		{
			ContinuousDetection detection;
			detection.Mode = ContinuousDetectionMode::Passive;
			detection.MinimumSweepTimestep = 0;
			detection.SweepConvergenceThreshold = 0;
			return detection;
		}

		/// <summary>
		/// <para>Collision detection will start with a sweep test to identify a likely time of impact. Speculative contacts will be generated for the predicted collision.</para>
		/// <para>This mode can capture angular motion with very few ghost collisions. It can, however, miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
		/// </summary>
		/// <param name="minimumSweepTimestep">Minimum progress that the sweep test will make when searching for the first time of impact.
		/// Collisions lasting less than MinimumProgress may be missed by the sweep test. Using larger values can significantly increase the performance of sweep tests.</param>
		/// <param name="sweepConvergenceThreshold">Threshold against which the time of impact region is compared for sweep termination. 
		/// If the region has been refined to be smaller than SweepConvergenceThreshold, the sweep will terminate.
		/// Values closer to zero will converge more closely to the true time of impact, but for speculative contact generation larger values usually work fine.
		/// Larger values allow the sweep to terminate much earlier and can significantly improve sweep performance.</param>
		/// <returns>Detection settings for the given continuous configuration.</returns>
		static ContinuousDetection Continuous(float minimumSweepTimestep = 1e-3f, float sweepConvergenceThreshold = 1e-3f)
		{
			ContinuousDetection detection;
			detection.Mode = ContinuousDetectionMode::Continuous;
			detection.MinimumSweepTimestep = minimumSweepTimestep;
			detection.SweepConvergenceThreshold = sweepConvergenceThreshold;
			return detection;
		}
	};

	/// <summary>
	/// Description of a collidable used by a body living in the broad phase and able to generate collision pairs.
	/// Collidables with a ShapeIndex that points to nothing (a default constructed <see cref="TypedIndex"/>) are not capable of colliding with anything.
	/// This can be used for a body which needs no collidable representation.
	/// </summary>
	struct Collidable
	{
		/// <summary>
		/// Index of the shape used by the body. While this can be changed, any transition from shapeless->shapeful or shapeful->shapeless must be reported to the broad phase. 
		/// If you need to perform such a transition, consider using <see cref="Bodies.SetShape"/> or <see cref="Bodies.ApplyDescription"/>; those functions update the relevant state.
		/// </summary>
		TypedIndex Shape;
		/// <summary>
		/// Continuous collision detection settings for this collidable. Includes the collision detection mode to use and tuning variables associated with those modes.
		/// </summary>
		ContinuousDetection Continuity;
		/// <summary>
		/// Lower bound on the value of the speculative margin used by the collidable.
		/// </summary>
		/// <remarks>0 tends to be a good default value. Higher values can be chosen if velocity magnitude is a poor proxy for speculative margins, but these cases are rare.
		/// In those cases, try to use the smallest value that still satisfies requirements to avoid creating unnecessary contact constraints.</remarks>
		float MinimumSpeculativeMargin;
		/// <summary>
		/// Upper bound on the value of the speculative margin used by the collidable.
		/// </summary>
		/// <remarks><see cref="float.MaxValue"/> tends to be a good default value for discrete or passive mode collidables. 
		/// The speculative margin will increase in size proportional to velocity magnitude, so having an unlimited maximum won't cost extra if the body isn't moving fast.
		/// <para>Smaller values can be useful for improving performance in chaotic situations where missing a collision is acceptable. When using <see cref="ContinuousDetectionMode.Continuous"/>, a speculative margin larger than the velocity magnitude will result in the sweep test being skipped, so lowering the maximum margin can help avoid ghost collisions.</para>
		/// </remarks>
		float MaximumSpeculativeMargin;

		/// <summary>
		/// Automatically computed size of the margin around the surface of the shape in which contacts can be generated. These contacts will have negative depth and only contribute if the frame's velocities
		/// would push the shapes of a pair into overlap. 
		/// <para>This is automatically set by bounding box prediction each frame, and is bound by the collidable's <see cref="MinimumSpeculativeMargin"/> and <see cref="MaximumSpeculativeMargin"/> values.
		/// The effective speculative margin for a collision pair can also be modified from <see cref="INarrowPhaseCallbacks"/> callbacks.</para>
		/// <para>This should be positive to avoid jittering.</para>
		/// <para>It can also be used as a form of continuous collision detection, but excessively high values combined with fast motion may result in visible 'ghost collision' artifacts. 
		/// For continuous collision detection with less chance of ghost collisions, use <see cref="ContinuousDetectionMode.Continuous"/>.</para>
		/// <para>If using <see cref="ContinuousDetectionMode.Continuous"/>, consider setting <see cref="MaximumSpeculativeMargin"/> to a smaller value to help filter ghost collisions.</para>
		/// <para>For more information, see the ContinuousCollisionDetection.md documentation.</para>
		/// </summary>
		float SpeculativeMargin;
		/// <summary>
		/// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters. Under normal circumstances, this should not be set externally.
		/// </summary>
		int32_t BroadPhaseIndex;
	};

	/// <summary>
	/// Describes how a body sleeps, and its current state with respect to sleeping.
	/// </summary>
	struct BodyActivity
	{
		/// <summary>
		/// Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).
		/// Setting this to a negative value guarantees the body cannot go to sleep without user action.
		/// </summary>
		float SleepThreshold;
		/// <summary>
		/// The number of time steps that the body must be under the sleep threshold before the body becomes a sleeping candidate.
		/// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.
		/// </summary>
		uint8_t MinimumTimestepsUnderThreshold;

		//Note that all values beyond this point are runtime set. The user should virtually never need to modify them. 
		//We do not constrain write access by default, instead opting to leave it open for advanced users to mess around with.
		//TODO: If people misuse these, we should internalize them in a case by case basis.

		/// <summary>
		/// If the body is awake, this is the number of time steps that the body has had a velocity below the sleep threshold.
		/// </summary>
		uint8_t TimestepsUnderThresholdCount;
		//Note that this flag is held alongside the other sleeping data, despite the fact that the traversal only needs the SleepCandidate state.
		//This is primarily for simplicity, but also note that the dominant accessor of this field is actually the sleep candidacy computation. Traversal doesn't visit every
		//body every frame, but sleep candidacy analysis does.
		//The reason why this flag exists at all is just to prevent traversal from being aware of the logic behind candidacy managemnt.
		//It doesn't cost anything extra to store this; it fits within the 8 byte layout.
		/// <summary>
		/// True if this body is a candidate for being slept. If all the bodies that it is connected to by constraints are also candidates, this body may go to sleep.
		/// </summary>
		bool SleepCandidate;
	};

	/// <summary>
	/// Describes a collidable and how it should handle continuous collision detection.
	/// </summary>
	struct CollidableDescription
	{
		/// <summary>
		/// Shape of the collidable.
		/// </summary>
		TypedIndex Shape;
		/// <summary>
		/// Continuous collision detection settings used by the collidable.
		/// </summary>
		ContinuousDetection Continuity;
		/// <summary>
		/// Lower bound on the value of the speculative margin used by the collidable.
		/// </summary>
		/// <remarks>0 tends to be a good default value. Higher values can be chosen if velocity magnitude is a poor proxy for speculative margins, but these cases are rare.
		/// In those cases, try to use the smallest value that still satisfies requirements to avoid creating unnecessary contact constraints.</remarks>
		float MinimumSpeculativeMargin;
		/// <summary>
		/// Upper bound on the value of the speculative margin used by the collidable.
		/// </summary>
		/// <remarks><see cref="float.MaxValue"/> tends to be a good default value for discrete or passive mode collidables. 
		/// The speculative margin will increase in size proportional to velocity magnitude, so having an unlimited maximum won't cost extra if the body isn't moving fast.
		/// <para>Smaller values can be useful for improving performance in chaotic situations where missing a collision is acceptable. When using <see cref="ContinuousDetectionMode.Continuous"/>, a speculative margin larger than the velocity magnitude will result in the sweep test being skipped, so lowering the maximum margin can help avoid ghost collisions.</para>
		/// </remarks>
		float MaximumSpeculativeMargin;

		/// <summary>
		/// Constructs a new collidable description.
		/// </summary>
		/// <param name="shape">Shape used by the collidable.</param>
		/// <param name="minimumSpeculativeMargin">Lower bound on the value of the speculative margin used by the collidable.</param>
		/// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
		/// <param name="continuity">Continuous collision detection settings for the collidable.</param>
		CollidableDescription(TypedIndex shape, float minimumSpeculativeMargin, float maximumSpeculativeMargin, ContinuousDetection continuity)
		{
			Shape = shape;
			MinimumSpeculativeMargin = minimumSpeculativeMargin;
			MaximumSpeculativeMargin = maximumSpeculativeMargin;
			Continuity = continuity;
		}

		/// <summary>
		/// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Discrete"/>.
		/// </summary>
		/// <param name="shape">Shape used by the collidable.</param>
		/// <param name="minimumSpeculativeMargin">Lower bound on the value of the speculative margin used by the collidable.</param>
		/// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
		CollidableDescription(TypedIndex shape, float minimumSpeculativeMargin, float maximumSpeculativeMargin)
		{
			Shape = shape;
			MinimumSpeculativeMargin = minimumSpeculativeMargin;
			MaximumSpeculativeMargin = maximumSpeculativeMargin;
			Continuity = ContinuousDetection::Discrete();
		}

		/// <summary>
		/// Constructs a new collidable description. Uses 0 for the <see cref="MinimumSpeculativeMargin"/> .
		/// </summary>
		/// <param name="shape">Shape used by the collidable.</param>
		/// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
		/// <param name="continuity">Continuous collision detection settings for the collidable.</param>
		CollidableDescription(TypedIndex shape, float maximumSpeculativeMargin, ContinuousDetection continuity)
		{
			Shape = shape;
			MinimumSpeculativeMargin = 0;
			MaximumSpeculativeMargin = maximumSpeculativeMargin;
			Continuity = continuity;
		}

		/// <summary>
		/// Constructs a new collidable description. Uses 0 for the <see cref="MinimumSpeculativeMargin"/> and <see cref="float.MaxValue"/> for the <see cref="MaximumSpeculativeMargin"/> .
		/// </summary>
		/// <param name="shape">Shape used by the collidable.</param>
		/// <param name="continuity">Continuous collision detection settings for the collidable.</param>
		CollidableDescription(TypedIndex shape, ContinuousDetection continuity)
		{
			Shape = shape;
			MinimumSpeculativeMargin = 0;
			MaximumSpeculativeMargin = std::numeric_limits<float>::max();
			Continuity = continuity;
		}

		/// <summary>
		/// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Passive"/>. Will use a <see cref="MinimumSpeculativeMargin"/> of 0 and a <see cref="MaximumSpeculativeMargin"/> of <see cref="float.MaxValue"/>.
		/// </summary>
		/// <param name="shape">Shape used by the collidable.</param>
		/// <remarks><see cref="ContinuousDetectionMode.Passive"/> and <see cref="ContinuousDetectionMode.Discrete"/> are equivalent in behavior when the <see cref="MaximumSpeculativeMargin"/>  is <see cref="float.MaxValue"/> since they both result in the same (unbounded) expansion of body bounding boxes in response to velocity.</remarks>
		CollidableDescription(TypedIndex shape) : CollidableDescription(shape, 0, std::numeric_limits<float>::max(), ContinuousDetection::Passive())
		{
		}

		/// <summary>
		/// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Discrete"/>. Will use a minimum speculative margin of 0 and the given maximumSpeculativeMargin.
		/// </summary>
		/// <param name="shape">Shape used by the collidable.</param>
		/// <param name="maximumSpeculativeMargin">Maximum speculative margin to be used with the discrete continuity configuration.</param>
		CollidableDescription(TypedIndex shape, float maximumSpeculativeMargin) : CollidableDescription(shape, 0, maximumSpeculativeMargin, ContinuousDetection::Discrete())
		{
		}
	};

	/// <summary>
	/// Describes the thresholds for a body going to sleep.
	/// </summary>
	struct BodyActivityDescription
	{
		/// <summary>
		/// Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).
		/// </summary>
		float SleepThreshold;
		/// <summary>
		/// The number of time steps that the body must be under the sleep threshold before the body becomes a sleep candidate.
		/// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.
		/// </summary>
		uint8_t MinimumTimestepCountUnderThreshold;

		/// <summary>
		/// Creates a body activity description.
		/// </summary>
		/// <param name="sleepThreshold">Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).</param>
		/// <param name="minimumTimestepCountUnderThreshold">The number of time steps that the body must be under the sleep threshold before the body becomes a sleep candidate.
		/// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.</param>
		BodyActivityDescription(float sleepThreshold, uint8_t minimumTimestepCountUnderThreshold = 32)
		{
			SleepThreshold = sleepThreshold;
			MinimumTimestepCountUnderThreshold = minimumTimestepCountUnderThreshold;
		}

		/// <summary>
		/// Creates a body activity description. Uses a <see cref="MinimumTimestepCountUnderThreshold"/> of 32.
		/// </summary>
		/// <param name="sleepThreshold">Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).</param>
		/// <param name="minimumTimestepCountUnderThreshold">The number of time steps that the body must be under the sleep threshold before the body becomes a sleep candidate.
		/// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.</param>
		BodyActivityDescription(float sleepThreshold) : BodyActivityDescription(sleepThreshold, 32)
		{
		}
	};

	/// <summary>
	/// Describes a body's state.
	/// </summary>
	struct BodyDescription
	{
		/// <summary>
		/// Position and orientation of the body.
		/// </summary>
		RigidPose Pose;
		/// <summary>
		/// Linear and angular velocity of the body.
		/// </summary>
		BodyVelocity Velocity;
		/// <summary>
		/// Mass and inertia tensor of the body.
		/// </summary>
		BodyInertia LocalInertia;
		/// <summary>
		/// Shape and collision detection settings for the body.
		/// </summary>
		CollidableDescription Collidable;
		/// <summary>
		/// Sleeping settings for the body.
		/// </summary>
		BodyActivityDescription Activity;

		/// <summary>
		/// Creates a dynamic body description.
		/// </summary>
		/// <param name="pose">Pose of the body.</param>
		/// <param name="velocity">Initial velocity of the body.</param>
		/// <param name="inertia">Local inertia of the body.</param>
		/// <param name="collidable">Collidable to associate with the body.</param>
		/// <param name="activity">Activity settings for the body.</param>
		/// <returns>Constructed description for the body.</returns>
		static BodyDescription CreateDynamic(RigidPose pose, BodyVelocity velocity, BodyInertia inertia, CollidableDescription collidable, BodyActivityDescription activity)
		{
			return BodyDescription{ pose, velocity, inertia, collidable, activity };
		}

		/// <summary>
		/// Creates a dynamic body description with zero initial velocity.
		/// </summary>
		/// <param name="pose">Pose of the body.</param>
		/// <param name="inertia">Local inertia of the body.</param>
		/// <param name="collidable">Collidable to associate with the body.</param>
		/// <param name="activity">Activity settings for the body.</param>
		/// <returns>Constructed description for the body.</returns>
		static BodyDescription CreateDynamic(RigidPose pose, BodyInertia inertia, CollidableDescription collidable, BodyActivityDescription activity)
		{
			return BodyDescription{ pose, {}, inertia, collidable, activity };
		}

		/// <summary>
		/// Creates a kinematic body description.
		/// </summary>
		/// <param name="pose">Pose of the body.</param>
		/// <param name="velocity">Initial velocity of the body.</param>
		/// <param name="collidable">Collidable to associate with the body.</param>
		/// <param name="activity">Activity settings for the body.</param>
		/// <returns>Constructed description for the body.</returns>
		static BodyDescription CreateKinematic(RigidPose pose, BodyVelocity velocity, CollidableDescription collidable, BodyActivityDescription activity)
		{
			return BodyDescription{ pose, velocity, {}, collidable, activity };
		}

		/// <summary>
		/// Creates a kinematic body description with zero initial velocity.
		/// </summary>
		/// <param name="pose">Pose of the body.</param>
		/// <param name="collidable">Collidable to associate with the body.</param>
		/// <param name="activity">Activity settings for the body.</param>
		/// <returns>Constructed description for the body.</returns>
		static BodyDescription CreateKinematic(RigidPose pose, CollidableDescription collidable, BodyActivityDescription activity)
		{
			return BodyDescription{ pose, {}, {}, collidable, activity };
		}
	};


	struct BodyConstraintReference
	{
		ConstraintHandle ConnectingConstraintHandle;
		int32_t BodyIndexInConstraint;
	};

	/// <summary>
	/// Describes the properties of a static object. When added to a simulation, static objects can collide but have no velocity and will not move in response to forces.
	/// </summary>
	struct StaticDescription
	{
		/// <summary>
		/// Position and orientation of the static.
		/// </summary>
		RigidPose Pose;
		/// <summary>
		/// Shape of the static.
		/// </summary>
		TypedIndex Shape;
		/// <summary>
		/// Continuous collision detection settings for the static.
		/// </summary>
		ContinuousDetection Continuity;

		/// <summary>
		/// Builds a new static description.
		/// </summary>
		/// <param name="pose">Pose of the static collidable.</param>
		/// <param name="shape">Shape of the static.</param>
		/// <param name="continuity">Continuous collision detection settings for the static.</param>
		StaticDescription(RigidPose pose, TypedIndex shape, ContinuousDetection continuity)
		{
			Pose = pose;
			Shape = shape;
			Continuity = continuity;
		}

		/// <summary>
		/// Builds a new static description with <see cref="ContinuousDetectionMode.Discrete"/> continuity.
		/// </summary>
		/// <param name="pose">Pose of the static collidable.</param>
		/// <param name="shape">Shape of the static.</param>
		StaticDescription(RigidPose pose, TypedIndex shape)
		{
			Pose = pose;
			Shape = shape;
			Continuity = ContinuousDetection::Discrete();
		}

		/// <summary>
		/// Builds a new static description.
		/// </summary>
		/// <param name="position">Position of the static.</param>
		/// <param name="orientation">Orientation of the static.</param>
		/// <param name="shape">Shape of the static.</param>
		/// <param name="continuity">Continuous collision detection settings for the static.</param>
		StaticDescription(Vector3 position, Quaternion orientation, TypedIndex shape, ContinuousDetection continuity)
		{
			Pose.Position = position;
			Pose.Orientation = orientation;
			Shape = shape;
			Continuity = continuity;
		}

		/// <summary>
		/// Builds a new static description with <see cref="ContinuousDetectionMode.Discrete"/> continuity.
		/// </summary>
		/// <param name="position">Position of the static.</param>
		/// <param name="orientation">Orientation of the static.</param>
		/// <param name="shape">Shape of the static.</param>
		StaticDescription(Vector3 position, Quaternion orientation, TypedIndex shape)
		{
			Pose.Position = position;
			Pose.Orientation = orientation;
			Shape = shape;
			Continuity = ContinuousDetection::Discrete();
		}
	};


	/// <summary>
	/// Stores data for a static collidable in the simulation. Statics can be posed and collide, but have no velocity and no dynamic behavior.
	/// </summary>
	/// <remarks>Unlike bodies, statics have a very simple access pattern. Most data is referenced together and there are no extreme high frequency data accesses like there are in the solver.
	/// Everything can be conveniently stored within a single location contiguously.</remarks>
	struct Static
	{
		/// <summary>
		/// Pose of the static collidable.
		/// </summary>
		RigidPose Pose;

		/// <summary>
		/// Continuous collision detection settings for this collidable. Includes the collision detection mode to use and tuning variables associated with those modes.
		/// </summary>
		/// <remarks>Note that statics cannot move, so there is no difference between <see cref="ContinuousDetectionMode.Discrete"/> and <see cref="ContinuousDetectionMode.Passive"/> for them.
		/// Enabling <see cref="ContinuousDetectionMode.Continuous"/> will still require that pairs associated with the static use swept continuous collision detection.</remarks>
		ContinuousDetection Continuity;

		/// <summary>
		/// Index of the shape used by the static. While this can be changed, any transition from shapeless->shapeful or shapeful->shapeless must be reported to the broad phase. 
		/// If you need to perform such a transition, consider using <see cref="Statics.SetShape"/> or Statics.ApplyDescription; those functions update the relevant state.
		/// </summary>
		TypedIndex Shape;
		//Note that statics do not store a 'speculative margin' independently of the contini
		/// <summary>
		/// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters. Under normal circumstances, this should not be set externally.
		/// </summary>
		int32_t BroadPhaseIndex;
	};


	/// <summary>
	/// Span over an unmanaged memory region.
	/// </summary>
	/// <typeparam name="T">Type of the memory exposed by the span.</typeparam>
	template<typename T>
	struct Buffer
	{
		/// <summary>
		/// Pointer to the beginning of the memory backing this buffer.
		/// </summary>
		T* Memory; //going to just assume 64 bit here.

		/// <summary>
		/// Length of the  to the beginning of the memory backing this buffer.
		/// </summary>
		int32_t Length;

		/// <summary>
		/// Implementation specific identifier of the raw buffer set by its source. If taken from a BufferPool, Id includes the index in the power pool from which it was taken.
		/// </summary>
		int32_t Id;

		T& operator[](int32_t index)
		{
			assert(index >= 0 && index < Length);
			return Memory[index];
		}
	};

	template<typename T>
	struct QuickList
	{
		/// <summary>
		/// Backing memory containing the elements of the list.
		/// Indices from 0 to Count-1 hold actual data. All other data is undefined.
		/// </summary>
		Buffer<T> Span;

		/// <summary>
		/// Number of elements in the list.
		/// </summary>
		int32_t Count;

		T& operator[](int32_t index)
		{
			assert(index >= 0 && index < Count);
			return Span[index];
		}
	};

	/// <summary>
	/// Represents how a collidable can interact and move.
	/// </summary>
	enum struct CollidableMobility : uint32_t
	{
		/// <summary>
		/// Marks a collidable as owned by a dynamic body.
		/// </summary>
		Dynamic = 0,
		/// <summary>
		/// Marks a collidable as owned by a kinematic body.
		/// </summary>
		Kinematic = 1,
		/// <summary>
		/// Marks the collidable as an independent immobile collidable.
		/// </summary>
		Static = 2
	};

	/// <summary>
	/// Uses a bitpacked representation to refer to a body or static collidable.
	/// </summary>
	struct CollidableReference
	{
		/// <summary>
		/// Bitpacked representation of the collidable reference.
		/// </summary>
		uint32_t Packed;

		/// <summary>
		/// Gets the mobility state of the owner of this collidable.
		/// </summary>
		CollidableMobility GetMobility()
		{
			return (CollidableMobility)(Packed >> 30);
		}

		/// <summary>
		/// Gets the body handle of the owner of the collidable referred to by this instance.
		/// </summary>
		BodyHandle GetBodyHandle()
		{
			assert(GetMobility() == CollidableMobility::Dynamic || GetMobility() == CollidableMobility::Kinematic, "Extracting a body handle from a collidable reference requires that the collidable is owned by a body.");
			return BodyHandle{ GetRawHandleValue() };

		}

		/// <summary>
		/// Gets the static handle of the owner of the collidable referred to by this instance.
		/// </summary>
		StaticHandle GetStaticHandle()
		{
			assert(GetMobility() == CollidableMobility::Static, "Extracting a static handle from a collidable reference requires that the collidable is owned by a static.");
			return StaticHandle{ GetRawHandleValue() };
		}

		/// <summary>
		/// Gets the integer value of the handle of the owner of the collidable referred to by this instance.
		/// </summary>
		int32_t GetRawHandleValue()
		{
			return (int)(Packed & 0x3FFFFFFF);
		}
	};

	struct CollidablePair
	{
		CollidableReference A;
		CollidableReference B;
	};

	struct NarrowPhaseCallbacks
	{
		void (*InitializeFunction)(SimulationHandle);
		void (*DisposeFunction)(SimulationHandle);
		bool (*AllowContactGenerationFunction)(SimulationHandle, int, CollidableReference, CollidableReference, float*);
		bool (*AllowContactGenerationBetweenChildrenFunction)(SimulationHandle, int, CollidablePair, int, int);
		bool (*ConfigureConvexContactManifoldFunction)(SimulationHandle, int, CollidablePair, ConvexContactManifold*, PairMaterialProperties*);
		bool (*ConfigureNonconvexContactManifoldFunction)(SimulationHandle, int, CollidablePair, NonconvexContactManifold*, PairMaterialProperties*);
		bool (*ConfigureChildContactManifoldFunction)(SimulationHandle, int, CollidablePair, int, int, ConvexContactManifold*);
	};

	extern "C" void Initialize();
	extern "C" void Destroy();
	extern "C" SIMDWidth GetSIMDWidth();
	extern "C" BufferPoolHandle CreateBufferPool(int32_t minimumBlockAllocationSize, int32_t expectedUsedSlotCountPerPool);
	extern "C" void ClearBufferPool(BufferPoolHandle handle);
	extern "C" void DestroyBufferPool(BufferPoolHandle handle);
	extern "C" ThreadDispatcherHandle CreateThreadDispatcher(int32_t threadCount, int32_t threadPoolAllocationBlockSize);
	extern "C" void DestroyThreadDispatcher(ThreadDispatcherHandle handle);
	extern "C" int32_t GetThreadCount(ThreadDispatcherHandle handle);
	extern "C" SimulationHandle CreateSimulation(BufferPoolHandle bufferPool, NarrowPhaseCallbacks narrowPhaseCallbacks, PoseIntegratorCallbacks poseIntegratorCallbacks, SolveDescription solveDescriptionInterop, SimulationAllocationSizes initialAllocationSizes);
	extern "C" void DestroySimulation(SimulationHandle handle);
	extern "C" BodyHandle AddBody(SimulationHandle simulationHandle, BodyDescription bodyDescription);
	extern "C" void RemoveBody(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyDynamics * GetBodyDynamics(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" Collidable * GetBodyCollidable(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyActivity * GetBodyActivity(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" QuickList<BodyConstraintReference> GetBodyConstraints(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" BodyDescription GetBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle);
	extern "C" void ApplyBodyDescription(SimulationHandle simulationHandle, BodyHandle bodyHandle, BodyDescription description);
	extern "C" StaticHandle AddStatic(SimulationHandle simulationHandle, StaticDescription staticDescription);
	extern "C" void RemoveStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" Static * GetStatic(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" StaticDescription GetStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle);
	extern "C" void ApplyStaticDescription(SimulationHandle simulationHandle, StaticHandle staticHandle, StaticDescription description);
	extern "C" void Timestep(SimulationHandle simulationHandle, float dt, ThreadDispatcherHandle threadDispatcherHandle);

}