using BEPUutilities2;
using DemoContentLoader;
using SharpFont;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace DemoContentBuilder
{
    public static class FontBuilder
    {
        private static Dictionary<CharacterPair, float> ComputeKerningTable(Face face, string characterSet)
        {
            var kerningTable = new Dictionary<CharacterPair, float>();
            for (int i = 0; i < characterSet.Length; ++i)
            {
                var glyphIndex = face.GetCharIndex(characterSet[i]);
                for (int j = 0; j < characterSet.Length; ++j)
                {
                    var kerning = face.GetKerning(glyphIndex, face.GetCharIndex(characterSet[i]), KerningMode.Default);
                    if (kerning.X != 0)
                    {
                        kerningTable.Add(new CharacterPair(characterSet[i], characterSet[j]), kerning.X.ToSingle());
                    }
                }
            }
            return kerningTable;
        }


        private const string characterSet = @"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz1234567890`-=[]\;',./~!@#$%^&*()_+{}|:""<>? ";

        private const int FontSizeInPixels = 128;
        private const int MipLevels = 4;
        private const int AtlasWidth = 2048;

        class CharacterHeightComparer : IComparer<CharacterData>
        {
            public int Compare(CharacterData a, CharacterData b)
            {
                return a.SourceSpan.Y > b.SourceSpan.Y ? 1 : a.SourceSpan.Y < b.SourceSpan.Y ? -1 : 0;
            }
        }

        public unsafe static FontContent Build(Stream fontDataStream)
        {
            var faceBytes = new byte[fontDataStream.Length];
            fontDataStream.Read(faceBytes, 0, faceBytes.Length);
            using (var library = new Library())
            {
                using (var face = new Face(library, faceBytes, 0))
                {

                    //Collect glyph boundings information.
                    face.SetPixelSizes(FontSizeInPixels, FontSizeInPixels);
                    var sortedCharacterSet = new char[characterSet.Length];
                    var sortedCharacterData = new CharacterData[characterSet.Length];

                    for (int i = 0; i < characterSet.Length; ++i)
                    {
                        sortedCharacterSet[i] = characterSet[i];
                        face.LoadGlyph(face.GetCharIndex(characterSet[i]), LoadFlags.Default, LoadTarget.Normal);
                        ref var characterData = ref sortedCharacterData[i];
                        characterData.SourceSpan.X = face.Glyph.Metrics.Width.ToSingle();
                        characterData.SourceSpan.Y = face.Glyph.Metrics.Height.ToSingle();

                        characterData.Bearing.X = face.Glyph.Metrics.HorizontalBearingX.ToSingle();
                        characterData.Bearing.Y = face.Glyph.Metrics.HorizontalBearingY.ToSingle();

                        characterData.Advance = face.Glyph.Metrics.HorizontalAdvance.ToSingle();
                    }

                    //Next, allocate space in the atlas for each character.
                    //Sort the characters by height, and then scan from one side of the atlas to the other placing characters.
                    //Once the other side is reached, flip directions and repeat. Continue until no more characters remain.
                    Array.Sort(sortedCharacterData, sortedCharacterSet, new CharacterHeightComparer());

                    const int padding = 1 << MipLevels;
                    var characters = new Dictionary<char, CharacterData>();
                    var packer = new FontPacker(AtlasWidth, padding, characterSet.Length);
                    for (int i = 0; i < sortedCharacterSet.Length; ++i)
                    {
                        //The packer class handles the placement logic and sets the SourceMinimum in the character data.
                        //So, after it is added to the intervals set, we can add the character-data pair to the font's dictionary.
                        packer.Add(ref sortedCharacterData[i]);
                        characters.Add(sortedCharacterSet[i], sortedCharacterData[i]);
                    }



                    //Now that every glyph has been positioned within the sheet, we can actually rasterize the glyph alphas into a bitmap proto-atlas.
                    //We're building the rasterized set sequentially first so we don't have to worry about threading issues in the underlying library.
                    var atlas = new Texture2DContent(AtlasWidth, packer.Height, MipLevels, 1);
                    var rasterizedAlphasTexture = new Texture2DContent(AtlasWidth, packer.Height, 1, 1);
                    for (int i = 0; i < sortedCharacterSet.Length; ++i)
                    {
                        //Rasterize the glyph.
                        var character = sortedCharacterSet[i];
                        face.LoadGlyph(face.GetCharIndex(character), LoadFlags.Default, LoadTarget.Normal);
                        face.Glyph.RenderGlyph(RenderMode.Normal);

                        //Copy the alphas into the pixel alpha buffer at the appropriate position.
                        int glyphWidth = face.Glyph.Bitmap.Width;
                        int glyphHeight = face.Glyph.Bitmap.Rows;
                        var glyphBuffer = (byte*)face.Glyph.Bitmap.Buffer;


                        Int2 location;
                        location.X = (int)sortedCharacterData[i].SourceMinimum.X;
                        location.Y = (int)sortedCharacterData[i].SourceMinimum.Y;
                        for (int glyphRow = 0; glyphRow < glyphHeight; ++glyphRow)
                        {
                            Unsafe.CopyBlockUnaligned(
                                ref rasterizedAlphasTexture.Data[atlas.GetRowByteOffsetFromMipStart(0, glyphRow + location.Y) + location.X],
                                ref glyphBuffer[glyphRow * glyphWidth], (uint)glyphWidth);
                        }
                    }

                    //Compute the distances for every character-covered texel in the atlas.
                    Parallel.For(0, characters.Count, i =>
                    {
                        //Note that the padding around characters should also have its distances filled in. That way, the less detailed mips can pull from useful data.

                        //Interpret partial alphas as distances.

                        //Convert the distance into a glyph normalized format.
                        //The glyph renderer will use this same normalization to interpret the distances.

                    });

                    //Build the mips.
                    //Note that while the padding avoids cross-character contamination, there's no reason to exit the character bounds when creating lower mips; that data
                    //might as well be at maximum distance.

                    //Build the kerning table.
                    var kerning = ComputeKerningTable(face, characterSet);

                    return new FontContent(atlas, face.FamilyName, 1f / FontSizeInPixels, characters, kerning);
                }
            }
        }




    }
}
