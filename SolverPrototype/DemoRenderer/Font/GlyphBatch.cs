﻿using System;
using System.Numerics;
using System.Text;

namespace DemoRenderer.Font
{
    /// <summary>
    /// Helper class to build renderable sets of glyphs.
    /// </summary>
    public class GlyphBatch
    {
        GlyphInstance[] glyphs;
        public GlyphInstance[] Glyphs
        {
            get
            {
                return glyphs;
            }
        }

        public int GlyphCount { get; private set; }

        public GlyphBatch(int initialCapacity = 128)
        {
            glyphs = new GlyphInstance[initialCapacity];
        }
            

        public void Clear()
        {
            GlyphCount = 0;
        }


        public void Add(StringBuilder characters, int start, int count, Vector2 startingPosition, Vector2 horizontalAxis, float height, Font font)
        {
            var scale = height * font.Content.InverseSizeInTexels;
            //Note that we don't actually include glyphs for spaces, so this could result in an oversized allocation. Not very concerning; no effect on correctness.
            var potentiallyRequiredCapacity = GlyphCount + count;
            if (potentiallyRequiredCapacity > glyphs.Length)
            {
                Array.Resize(ref glyphs, Math.Max(potentiallyRequiredCapacity, glyphs.Length * 2));
            }

            var penPosition = startingPosition;
            var verticalAxis = new Vector2(-horizontalAxis.Y, horizontalAxis.X);
            var nextCharacterIndex = start;
            var characterEnd = start + count;
            while (true)
            {
                if (nextCharacterIndex >= characterEnd)
                    break;
                var character = characters[nextCharacterIndex++];
                //Only create a glyph for characters that our font actually has entries for. Others will just be skipped without advancing.
                if (font.Content.Characters.TryGetValue(character, out var characterData))
                {
                    //No point wasting time rendering a glyph with nothing in it. At least for any normal-ish font.
                    if (character != ' ')
                    {
                        ref var glyph = ref glyphs[GlyphCount++];
                        //Note subtraction on y component. In texture space, +1 is down, -1 is up.
                        var localOffsetToCharacter = new Vector2(characterData.Bearing.X * scale, characterData.Bearing.Y * scale); 
                        var offsetToCharacter = localOffsetToCharacter.X * horizontalAxis - localOffsetToCharacter.Y * verticalAxis;
                        glyph.TargetPosition = penPosition + offsetToCharacter;
                        glyph.SourceId = font.GetSourceId(character);
                        glyph.Scale = scale;

                    }
                    //Move the pen to the next character.
                    float advance = characterData.Advance;
                    if (nextCharacterIndex < characterEnd)
                        advance += font.Content.GetKerningInTexels(character, characters[nextCharacterIndex]);
                    penPosition += horizontalAxis * (scale * advance);
                }
            }
        }
    }
}