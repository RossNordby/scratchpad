﻿using System;
using System.Diagnostics;
using SharpDX;
using SharpDX.Direct3D11;
using Buffer = SharpDX.Direct3D11.Buffer;
using System.Runtime.CompilerServices;
using System.Numerics;
using System.Runtime.InteropServices;

namespace DemoRenderer
{
    public static class Helpers
    {
        /// <summary>
        /// Sets an index buffer using a binding.
        /// </summary>
        /// <param name="inputAssembler">Assembler to set.</param>
        /// <param name="binding">Binding to set to the device for indices.</param>
        /// <param name="offset">Offset from the beginning of the index buffer to start reading.</param>
        public static void SetIndexBuffer(this InputAssemblerStage inputAssembler, IndexBuffer binding, int offset = 0)
        {
            inputAssembler.SetIndexBuffer(binding.Buffer, binding.Format, offset);
        }


        /// <summary>
        /// Updates a buffer using MapSubresource..
        /// </summary>
        /// <typeparam name="T">Type of the elements in the buffer.</typeparam>
        /// <param name="context">Context used to update the buffer.</param>
        /// <param name="buffer">Buffer to update.</param>
        /// <param name="newValues">Values to upload into the buffer.</param>
        /// <param name="sourceOffset">Starting index in the new values array to read from.</param>
        /// <param name="count">Number of elements in the values to upload into the buffer.</param>
        /// <param name="destinationOffset">Offset from the beginning of the buffer to store the new values.</param>
        public static void MapUpdateBuffer<T>(this DeviceContext context, Buffer buffer, T[] newValues, int count, int sourceOffset = 0, int destinationOffset = 0) where T : struct
        {
            var dataBox = context.MapSubresource(buffer, 0, MapMode.WriteDiscard, MapFlags.None);
            unsafe
            {
                var bytesPointer = (byte*)dataBox.DataPointer;
                bytesPointer += destinationOffset * Utilities.SizeOf<T>();
                Utilities.Write(new IntPtr(bytesPointer), newValues, sourceOffset, count);
            }
            context.UnmapSubresource(buffer, 0);
        }


        /// <summary>
        /// Updates a buffer using UpdateSubresource.
        /// </summary>
        /// <typeparam name="T">Type of the elements in the buffer.</typeparam>
        /// <param name="context">Context used to update the buffer.</param>
        /// <param name="buffer">Buffer to update.</param>
        /// <param name="newValues">Values to upload into the buffer.</param>
        /// <param name="sourceOffset">Starting index in the new values array to read from.</param>
        /// <param name="count">Number of elements in the values to upload into the buffer.</param>
        /// <param name="destinationOffset">Offset from the beginning of the buffer to store the new values.</param>
        public static unsafe void UpdateBuffer<T>(this DeviceContext context, Buffer buffer, T[] newValues, int count, int sourceOffset = 0, int destinationOffset = 0) where T : struct
        {
            var handle = GCHandle.Alloc(newValues, GCHandleType.Pinned);
            var bytes = (byte*)handle.AddrOfPinnedObject();
            var strideInBytes = Unsafe.SizeOf<T>();
            context.UpdateSubresource(
                new DataBox(new IntPtr(bytes + sourceOffset * strideInBytes)), buffer, 0,
                new ResourceRegion(
                    destinationOffset * strideInBytes, 0, 0,
                    (destinationOffset + count) * strideInBytes, 1, 1));
            handle.Free();
        }

        /// <summary>
        /// Creates an index buffer of the specified size for screenspace quads.
        /// </summary>
        /// <param name="quadCount">Number of quads to create indices for.</param>
        /// <returns>Index buffer for screenspace quads.</returns> 
        /// <remarks>Using redundant indices for batches avoids a slow path for low triangle count instancing. This is hardware/driver specific; it may change on newer cards.</remarks>
        public static uint[] GetQuadIndices(int quadCount)
        {
            var indexData = new uint[quadCount * 6];
            uint baseVertex = 0;
            for (int glyphIndexStart = 0; glyphIndexStart < indexData.Length; glyphIndexStart += 6)
            {
                //Front facing triangles are counter clockwise.
                //Quad layout: 
                // 0____1
                // |  / |
                // | /  |
                // 2____3
                //0 2 1
                //1 2 3 
                indexData[glyphIndexStart + 0] = baseVertex + 0;
                indexData[glyphIndexStart + 1] = baseVertex + 2;
                indexData[glyphIndexStart + 2] = baseVertex + 1;
                indexData[glyphIndexStart + 3] = baseVertex + 1;
                indexData[glyphIndexStart + 4] = baseVertex + 2;
                indexData[glyphIndexStart + 5] = baseVertex + 3;
                baseVertex += 4;
            }
            return indexData;
        }
        /// <summary>
        /// Creates an index buffer of the specified size for boxes.
        /// </summary>
        /// <param name="quadCount">Number of boxes to create indices for.</param>
        /// <returns>Index buffer for boxes.</returns> 
        /// <remarks>Using redundant indices for batches avoids a slow path for low triangle count instancing. This is hardware/driver specific; it may change on newer cards.</remarks>
        public static uint[] GetBoxIndices(int boxCount)
        {
            //Build a AABB mesh's index buffer, repeated. A redundant index buffer tends to be faster than instancing tiny models. Impact varies between hardware.
            var indexData = new uint[boxCount * 36];
            uint baseVertex = 0;
            for (int glyphIndexStart = 0; glyphIndexStart < indexData.Length; glyphIndexStart += 36)
            {
                //Note that the order and winding is important and must be consistent with the RenderSpheres.hlsl VS usage.
                //It assumes that an unset bit in the 3-bit string of the vertex id corresponds to the minimum position: 
                //vertex id 0 becomes (-1, -1, -1), while vertex id 7 becomes (1, 1, 1).
                //-X
                indexData[glyphIndexStart + 0] = baseVertex + 0;
                indexData[glyphIndexStart + 1] = baseVertex + 4;
                indexData[glyphIndexStart + 2] = baseVertex + 6;
                indexData[glyphIndexStart + 3] = baseVertex + 6;
                indexData[glyphIndexStart + 4] = baseVertex + 2;
                indexData[glyphIndexStart + 5] = baseVertex + 0;
                //+X
                indexData[glyphIndexStart + 6] = baseVertex + 5;
                indexData[glyphIndexStart + 7] = baseVertex + 1;
                indexData[glyphIndexStart + 8] = baseVertex + 3;
                indexData[glyphIndexStart + 9] = baseVertex + 3;
                indexData[glyphIndexStart + 10] = baseVertex + 7;
                indexData[glyphIndexStart + 11] = baseVertex + 5;
                //-Y
                indexData[glyphIndexStart + 12] = baseVertex + 0;
                indexData[glyphIndexStart + 13] = baseVertex + 1;
                indexData[glyphIndexStart + 14] = baseVertex + 5;
                indexData[glyphIndexStart + 15] = baseVertex + 5;
                indexData[glyphIndexStart + 16] = baseVertex + 4;
                indexData[glyphIndexStart + 17] = baseVertex + 0;
                //+Y
                indexData[glyphIndexStart + 18] = baseVertex + 2;
                indexData[glyphIndexStart + 19] = baseVertex + 6;
                indexData[glyphIndexStart + 20] = baseVertex + 7;
                indexData[glyphIndexStart + 21] = baseVertex + 7;
                indexData[glyphIndexStart + 22] = baseVertex + 3;
                indexData[glyphIndexStart + 23] = baseVertex + 2;
                //-Z
                indexData[glyphIndexStart + 24] = baseVertex + 1;
                indexData[glyphIndexStart + 25] = baseVertex + 0;
                indexData[glyphIndexStart + 26] = baseVertex + 2;
                indexData[glyphIndexStart + 27] = baseVertex + 2;
                indexData[glyphIndexStart + 28] = baseVertex + 3;
                indexData[glyphIndexStart + 29] = baseVertex + 1;
                //+Z
                indexData[glyphIndexStart + 30] = baseVertex + 4;
                indexData[glyphIndexStart + 31] = baseVertex + 5;
                indexData[glyphIndexStart + 32] = baseVertex + 7;
                indexData[glyphIndexStart + 33] = baseVertex + 7;
                indexData[glyphIndexStart + 34] = baseVertex + 6;
                indexData[glyphIndexStart + 35] = baseVertex + 4;
                baseVertex += 8;
            }
            return indexData;
        }
        /// <summary>
        /// Packs an RGB color in a UNORM manner such that bits 0 through 10 are R, bits 11 through 21 are G, and bits 22 through 31 are B. 
        /// </summary>
        /// <param name="color">RGB color to pack.</param>
        /// <returns>Color packed into 32 bits.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint PackColor(ref Vector3 color)
        {
            const uint RScale = (1 << 11) - 1;
            const uint GScale = (1 << 11) - 1;
            const uint BScale = (1 << 10) - 1;
            var r = (uint)(color.X * RScale);
            var g = (uint)(color.Y * GScale);
            var b = (uint)(color.Z * BScale);
            if (r > RScale)
                r = RScale;
            if (g > GScale)
                g = GScale;
            if (b > BScale)
                b = BScale;
            return r | (g << 11) | (b << 22);
        }

        [Conditional("DEBUG")]
        public static void CheckForUndisposed(bool disposed, object o)
        {
            Debug.Assert(disposed, "An object of type " + o.GetType() + " was not disposed prior to finalization.");
        }

        public static void Dispose<T>(ref T disposable) where T : IDisposable
        {
            if (disposable != null)
                disposable.Dispose();
            disposable = default(T);
        }

    }


}
