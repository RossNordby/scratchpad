using BEPUutilities2;
using BEPUutilities2.Memory;
using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace DemoRenderer.Font
{
    /// <summary>
    /// Accumulates text for rendering.
    /// </summary>
    public class TextBatcher
    {
        /// <summary>
        /// The renderer has to split up draw calls between glyphs of different fonts, colors, and rotations. Within that requirement, try to keep glyphs together.
        /// </summary>
        struct BatchDescription : IEquatable<BatchDescription>
        {
            public Font Font;
            public Vector2 HorizontalAxis;
            public Vector3 Color;

            public override int GetHashCode()
            {
                //This hash code could be better, but eh, it's fine.
                return (Font.GetHashCode() * 1811) ^ (HorizontalAxis.GetHashCode() * 3407) ^ (Color.GetHashCode() * 4937);
            }

            public bool Equals(BatchDescription other)
            {
                return Font == other.Font && HorizontalAxis == other.HorizontalAxis && Color == other.Color;
            }

            public override bool Equals(object obj)
            {
                return Equals((BatchDescription)obj);
            }
        }
        Pool<GlyphsBatch> batchPool = new Pool<GlyphsBatch>(() => new GlyphsBatch(), cleaner: batch => batch.Clear());

        Dictionary<BatchDescription, GlyphsBatch> batches = new Dictionary<BatchDescription, GlyphsBatch>();

        public void Write(StringBuilder characters, int start, int count, Vector2 targetPosition, float height,
            Vector2 horizontalAxis, Vector3 color, Font font)
        {
            var batchDescription = new BatchDescription { Font = font, Color = color, HorizontalAxis = horizontalAxis };
            if (!batches.TryGetValue(batchDescription, out var glyphBatch))
            {
                glyphBatch = batchPool.Take();
                batches.Add(batchDescription, glyphBatch);
            }

            glyphBatch.Add(characters, start, count, targetPosition, horizontalAxis, height, font);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Write(StringBuilder characters, Vector2 targetPosition, float height, Vector3 color, Font font)
        {
            Write(characters, 0, characters.Length, targetPosition, height, new Vector2(1, 0), color, font);
        }

        public void Flush(DeviceContext context, Int2 screenResolution, GlyphRenderer renderer)
        {
            foreach (var batch in batches)
            {
                renderer.Render(context, batch.Key.Font, screenResolution, batch.Key.HorizontalAxis, batch.Key.Color,
                    batch.Value.Glyphs, 0, batch.Value.GlyphCount);
                batchPool.Return(batch.Value);
            }
            batches.Clear();

        }
    }
}
