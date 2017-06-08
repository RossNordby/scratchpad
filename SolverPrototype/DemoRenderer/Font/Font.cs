using DemoContentLoader;
using SharpDX;
using SharpDX.Direct3D11;
using System;
using System.Text;

namespace DemoRenderer.Font
{
    /// <summary>
    /// Runtime type containing GPU-related information necessary to render a specific font type.
    /// </summary>
    public class Font : IDisposable
    {
        public StructuredBuffer<GlyphSource> Sources { get; private set; }
        public Texture2D Atlas { get; private set; }
        public ShaderResourceView AtlasSRV { get; private set; }

        public FontContent Content { get; private set; }

        public unsafe Font(Device device, DeviceContext context, FontContent font)
        {
            this.Content = font;
            Sources = new StructuredBuffer<GlyphSource>(device, font.GlyphCount, font.Name + " Glyph Sources");
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

            Sources.Update(context, font.GlyphSources);
        }

        public float MeasureTextLength(float height, StringBuilder characters)
        {
            return MeasureTextLength(height, characters, 0, characters.Length);
        }

        public float MeasureTextLength(float height, StringBuilder characters, int characterCount)
        {
            return MeasureTextLength(height, characters, 0, characterCount);
        }

        public float MeasureTextLength(float height, StringBuilder characters, int start, int characterCount)
        {
            //Compute the number of pixels per world space unit to convert pixel dimensions into world half-dimensions.
            int length = 0;
            var lastCharacter = characterCount - 1;
            for (int characterIndex = start; characterIndex < lastCharacter; ++characterIndex)
            {
                length += MeasureAdvance(characters[characterIndex], characters[characterIndex + 1]);
            }
            length += MeasureAdvance(characters[lastCharacter]);
            return length * (height * Content.InverseSizeInTexels);
        }

        /// <summary>
        /// Measures the distance from the beginning of the current character to the beginning of the next character in source texels.
        /// </summary>
        public int MeasureAdvance(char current, char next)
        {
            //Move to the next character's starting point.
            return Content.GetAdvanceInTexels(current) + Content.GetKerningInTexels(current, next);
        }
        /// <summary>
        /// Measures the distance from the beginning of the current character to the end of it.
        /// </summary>
        public int MeasureAdvance(char current)
        {
            return Content.GetAdvanceInTexels(current);
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
    }
}
