using System;
using System.Diagnostics;
using SharpDX;
using SharpDX.Direct3D11;
using Buffer = SharpDX.Direct3D11.Buffer;
using System.Runtime.CompilerServices;

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
        public static void UpdateBuffer<T>(this DeviceContext context, Buffer buffer, T[] newValues, int count, int sourceOffset = 0, int destinationOffset = 0) where T : struct
        {
            Utilities.Pin(newValues, ptr =>
            {
                unsafe
                {
                    var strideInBytes = Unsafe.SizeOf<T>();
                    var bytePtr = (byte*)ptr;
                    context.UpdateSubresource(
                        new DataBox(new IntPtr(bytePtr + sourceOffset * strideInBytes)), buffer, 0,
                        new ResourceRegion(
                            destinationOffset * strideInBytes, 0, 0,
                            (destinationOffset + count) * strideInBytes, 1, 1));
                }
            });
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
