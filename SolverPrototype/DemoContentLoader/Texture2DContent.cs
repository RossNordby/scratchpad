using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace DemoContentLoader
{
    public unsafe class Texture2DContent
    {
        public int Width { get; private set; }
        public int Height { get; private set; }
        public int MipLevels { get; private set; }
        public int TexelSizeInBytes { get; private set; }
        public int GetRowPitch(int mipLevel)
        {
            return TexelSizeInBytes * (Width >> mipLevel);
        }

        GCHandle handle;
        public byte[] Data { get; private set; }

        public Texture2DContent(int width, int height, int mipLevels, int texelSizeInBytes)
        {
            Width = width;
            Height = height;
            MipLevels = mipLevels;
            TexelSizeInBytes = texelSizeInBytes;
            var dataSize = 0;
            for (int i = 0; i < mipLevels; ++i)
            {
                dataSize += texelSizeInBytes * (width >> i) * (height >> i);
            }
            Data = new byte[dataSize];
        }
        
        public int GetMipStartByteIndex(int mipLevel)
        {
            int start = 0;
            for (int i = 0; i < mipLevel; ++i)
            {
                start += TexelSizeInBytes * (Width >> i) * (Height >> i);
            }
            return start;
        }

        public int GetRowByteOffsetFromMipStart(int mipLevel, int rowIndex)
        {
            return GetRowPitch(mipLevel) * rowIndex;
        }

        public byte* Pin()
        {
            if (handle.IsAllocated)
                throw new InvalidOperationException("Cannot pin an already-pinned texture.");
            handle = GCHandle.Alloc(Data, GCHandleType.Pinned);
            return (byte*)handle.AddrOfPinnedObject();
        }


        public void Unpin()
        {
            if (!handle.IsAllocated)
                throw new InvalidOperationException("Should only unpin textures that have been pinned.");
            handle.Free();
        }

#if DEBUG
        ~Texture2DContent()
        {
            Debug.Assert(!handle.IsAllocated, "Any resource getting finalized should no longer be pinned in memory. Check for Pins without Unpins.");
        }
#endif
    }
}
