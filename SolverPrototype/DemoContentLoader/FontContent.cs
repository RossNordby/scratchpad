using DemoContentLoader;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace DemoContentLoader
{
    /// <summary>
    /// Location of a glyph in the atlas.
    /// </summary>
    public struct GlyphSource
    {
        public Vector2 Minimum;
        public Vector2 Span;
    }

    public struct CharacterPair : IEquatable<CharacterPair>
    {
        public readonly char A;
        public readonly char B;

        public CharacterPair(char a, char b)
        {
            A = a;
            B = b;
        }

        //Order matters!
        public bool Equals(CharacterPair other)
        {
            return A == other.A && B == other.B;
        }

        public override int GetHashCode()
        {
            return (A * 7919) ^ (B * 6263);
        }

        public override string ToString()
        {
            return "{" + A + ", " + B + "}";
        }
    }
    public class FontContent
    {
        public int GlyphCount { get; private set; }
        public Texture2DContent Atlas { get; private set; }
        public string Name { get; private set; }
        public GlyphSource[] GlyphSources { get; private set; }
        public float InverseSizeInTexels { get; private set; }

        Dictionary<char, int> advances;
        Dictionary<CharacterPair, int> kerning;

        public FontContent(int glyphCount, Texture2DContent atlas, string name, GlyphSource[] sources, float inverseSizeInTexels,
            Dictionary<char, int> advances, Dictionary<CharacterPair, int> kerningTable)
        {
            GlyphCount = GlyphCount;
            Atlas = atlas;
            Name = name;
            GlyphSources = GlyphSources;
            InverseSizeInTexels = inverseSizeInTexels;
            this.advances = advances;
            this.kerning = kerningTable;
        }

        public int GetAdvanceInTexels(char character)
        {
            if (advances.TryGetValue(character, out var advance))
            {
                return advance;
            }
            return 0;
        }

        public int GetKerningInTexels(char a, char b)
        {
            if (kerning.TryGetValue(new CharacterPair(a, b), out var pairKerning))
            {
                return pairKerning;
            }
            return 0;
        }
    }
}
