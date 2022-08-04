#pragma once

#include <assert.h>
#include <stdint.h>
#include "Math.h"
#include "Handles.h"
#include "Constraints.h"

namespace Bepu
{
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

	/// <summary>
	/// Information about a single contact in a convex collidable pair. Convex collidable pairs share one surface basis across the manifold, since the contact surface is guaranteed to be a plane.
	/// </summary>    
	struct ConvexContact
	{
		/// <summary>
		/// Offset from the position of collidable A to the contact position. 
		/// </summary>
		Vector3 Offset;
		/// <summary>
		/// Penetration depth between the two collidables at this contact. Negative values represent separation.
		/// </summary>
		float Depth;
		/// <summary>
		/// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
		/// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
		/// </summary>
		int32_t FeatureId;
	};

	/// <summary>
	/// Contains the data associated with a convex contact manifold.
	/// </summary>
	struct ConvexContactManifold
	{
		/// <summary>
		/// Offset from collidable A to collidable B.
		/// </summary>
		Vector3 OffsetB;
		int32_t Count;

		/// <summary>
		/// Surface normal shared by all contacts. Points from collidable B to collidable A.
		/// </summary>
		Vector3 Normal;

		ConvexContact Contacts[4];

		void ValidateIndex(int contactIndex)
		{
			assert(contactIndex >= 0 && contactIndex < Count, "Contact index must be within the contact count.");
		}
	};

	/// <summary>
	/// Information about a single contact in a nonconvex collidable pair.
	/// Nonconvex pairs can have different surface bases at each contact point, since the contact surface is not guaranteed to be a plane.
	/// </summary>
	struct NonconvexContact
	{
		/// <summary>
		/// Offset from the position of collidable A to the contact position. 
		/// </summary>
		Vector3 Offset;
		/// <summary>
		/// Penetration depth between the two collidables at this contact. Negative values represent separation.
		/// </summary>
		float Depth;
		/// <summary>
		/// Surface basis of the contact. If transformed into a rotation matrix, X and Z represent tangent directions and Y represents the contact normal. Points from collidable B to collidable A.
		/// </summary>
		Vector3 Normal;
		/// <summary>
		/// Id of the features involved in the collision that generated this contact. If a contact has the same feature id as in a previous frame, it is an indication that the
		/// same parts of the shape contributed to its creation. This is useful for carrying information from frame to frame.
		/// </summary>
		int32_t FeatureId;
	};

	/// <summary>
	/// Contains the data associated with a nonconvex contact manifold.
	/// </summary>
	struct NonconvexContactManifold
	{
		/// <summary>
		/// Offset from collidable A to collidable B.
		/// </summary>
		Vector3 OffsetB;
		int32_t Count;

		NonconvexContact Contacts[4];
	};

	/// <summary>
	/// Material properties governing the interaction between colliding bodies. Used by the narrow phase to create constraints of the appropriate configuration.
	/// </summary>
	struct PairMaterialProperties
	{
		/// <summary>
		/// Coefficient of friction to apply for the constraint. Maximum friction force will be equal to the normal force times the friction coefficient.
		/// </summary>
		float FrictionCoefficient;
		/// <summary>
		/// Maximum relative velocity along the contact normal at which the collision constraint will recover from penetration. Clamps the velocity goal created from the spring settings.
		/// </summary>
		float MaximumRecoveryVelocity;
		/// <summary>
		/// Defines the constraint's penetration recovery spring properties.
		/// </summary>
		SpringSettings ContactSpringSettings;

		/// <summary>
		/// Constructs a pair's material properties.
		/// </summary>
		/// <param name="frictionCoefficient">Coefficient of friction to apply for the constraint. Maximum friction force will be equal to the normal force times the friction coefficient.</param>
		/// <param name="maximumRecoveryVelocity">Maximum relative velocity along the contact normal at which the collision constraint will recover from penetration. Clamps the velocity goal created from the spring settings.</param>
		/// <param name="springSettings">Defines the constraint's penetration recovery spring properties.</param>
		PairMaterialProperties(float frictionCoefficient, float maximumRecoveryVelocity, SpringSettings springSettings)
		{
			FrictionCoefficient = frictionCoefficient;
			MaximumRecoveryVelocity = maximumRecoveryVelocity;
			ContactSpringSettings = springSettings;
		}
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

}