#pragma once

#include <stdint.h>

namespace Bepu
{
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
		int32_t GetType() { return (int32_t)(Packed & 0x7F000000) >> 24; }

		/// <summary>
		/// Gets the index of the object.
		/// </summary>
		int32_t GetIndex() { return (int32_t)(Packed & 0x00FFFFFF); }

		/// <summary>
		/// Gets whether this index actually refers to anything. The Type and Index should only be used if this is true.
		/// </summary>
		bool Exists() { return (Packed & (1 << 31)) > 0; }
	};

	/// <summary>
	/// Points to an instance in an instance directory.
	/// </summary>
	struct InstanceHandle
	{
		int32_t RawValue;

		int32_t GetIndex() { return RawValue & 0x00FFFFFF; }
		int32_t GetVersion() { return (RawValue >> 24) & 0xF; }
		int32_t GetTypeIndex() { return (RawValue >> 28) & 0x7; }

		bool IsNull() { return RawValue == 0; }

		InstanceHandle()
		{
			RawValue = 0;
		}

		InstanceHandle(int index, int version, int typeIndex)
		{
			assert(index < (1 << 24), "This handle assumes there are less than 2^24 instances. There really should be less than a few dozen. Something is probably wrong.");
			assert(typeIndex < 8, "This handle assumes there are less than 8 types being registered into instance directories. Bepuphysics2 doesn't need many; if there's more, something may be wrong or this may need to be changed.");
			RawValue = (1 << 31) | index | (version << 24) | (typeIndex << 28);
		}
	};

	typedef int32_t BodyHandle;
	typedef int32_t StaticHandle;
	typedef int32_t ConstraintHandle;
	typedef InstanceHandle SimulationHandle;
	typedef InstanceHandle BufferPoolHandle;
	typedef InstanceHandle ThreadDispatcherHandle;
}