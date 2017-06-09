using DemoContentLoader;
using SharpDX;
using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace DemoRenderer.Font
{
    /// <summary>
    /// Location of a glyph in the atlas.
    /// </summary>
    public struct GlyphSource
    {
        public Vector2 Minimum;
        public Vector2 Span;
    }
    /// <summary>
    /// Runtime type containing GPU-related information necessary to render a specific font type.
    /// </summary>
    public class Font : IDisposable
    {
        public StructuredBuffer<GlyphSource> Sources { get; private set; }
        public Texture2D Atlas { get; private set; }
        public ShaderResourceView AtlasSRV { get; private set; }

        public FontContent Content { get; private set; }

        //Technically you could establish the char-source relationship within the font content itself, and that would eliminate one dictionary lookup.
        //However, source ids don't really exist outside of the runtime type, and establishing a consistent order for them would require a little more complexity.
        //Just doing it here is a little simpler. You can change this up if glyph setup is somehow ever a performance concern.
        Dictionary<char, int> sourceIds;

        public unsafe Font(Device device, DeviceContext context, FontContent font)
        {
            this.Content = font;
            Sources = new StructuredBuffer<GlyphSource>(device, font.Characters.Count, font.Name + " Glyph Sources");
            Atlas = new Texture2D(device, new Texture2DDescription
            {
                ArraySize = 1,
                BindFlags = BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.None,
                Format = SharpDX.DXGI.Format.A8_UNorm,
                Height = font.Atlas.Height,
                Width = font.Atlas.Width,
                MipLevels = font.Atlas.MipLevels,
                OptionFlags = ResourceOptionFlags.None,
                SampleDescription = new SharpDX.DXGI.SampleDescription(1, 0),
                Usage = ResourceUsage.Default
            });
            Atlas.DebugName = font.Name + " Atlas";
            AtlasSRV = new ShaderResourceView(device, Atlas);
            AtlasSRV.DebugName = font.Name + " Atlas SRV";

            var data = font.Atlas.Pin();
            for (int mipLevel = 0; mipLevel < font.Atlas.MipLevels; ++mipLevel)
            {
                var databox = new DataBox(new IntPtr(data + font.Atlas.GetMipStartByteIndex(mipLevel)), font.Atlas.GetRowPitch(mipLevel), 0);
                context.UpdateSubresource(databox, Atlas, mipLevel);
            }
            font.Atlas.Unpin();

            sourceIds = new Dictionary<char, int>();
            int nextSourceId = 0;
            var sourcesData = new GlyphSource[font.Characters.Count];
            foreach (var character in font.Characters)
            {
                sourceIds.Add(character.Key, nextSourceId);
                sourcesData[nextSourceId] = new GlyphSource { Minimum = character.Value.SourceMinimum, Span = character.Value.SourceSpan };
                ++nextSourceId;
            }
            Sources.Update(context, sourcesData);
        }

        public int GetSourceId(char character)
        {
            if (sourceIds.TryGetValue(character, out var sourceId))
            {
                return sourceId;
            }
            return -1;
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Sources.Dispose();
                Atlas.Dispose();
                AtlasSRV.Dispose();
            }
        }

#if DEBUG
        ~Font()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
