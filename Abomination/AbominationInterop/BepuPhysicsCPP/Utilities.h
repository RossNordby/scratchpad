#pragma once

#include <stdint.h>

namespace Bepu
{
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
}