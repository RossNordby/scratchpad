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

	typedef int32_t BodyHandle;
	typedef int32_t StaticHandle;
	typedef int32_t ConstraintHandle;
	typedef int32_t SimulationHandle;
	typedef int32_t BufferPoolHandle;
	typedef int32_t ThreadDispatcherHandle;
}