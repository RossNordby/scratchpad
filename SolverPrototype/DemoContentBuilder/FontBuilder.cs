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
                    var rasterizedAlphas = new Texture2DContent(AtlasWidth, packer.Height, 1, 1);
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
                                ref rasterizedAlphas.Data[atlas.GetRowByteOffsetFromMipStart(0, glyphRow + location.Y) + location.X],
                                ref glyphBuffer[glyphRow * glyphWidth], (uint)glyphWidth);
                        }
                    }

                    //Initialize the atlas distances to all 255. The BFS only changes values when the new distance is lower than the current distance.
                    //Note that the mip levels beyond 0 don't need to be initialized; they're fully filled later.
                    Unsafe.InitBlock(ref atlas.Data[0], 0xFF, (uint)atlas.GetMipStartByteIndex(1));

                    //Compute the distances for every character-covered texel in the atlas.
                    var atlasData = atlas.Pin();
                    var alphaData = rasterizedAlphas.Pin();
                    Parallel.For(0, sortedCharacterData.Length, i =>
                    {
                        //Note that the padding around characters should also have its distances filled in. That way, the less detailed mips can pull from useful data.
                        ref var charData = ref sortedCharacterData[i];

                        //Note that the conversion from distance to texels involves a normalization by the character's span. That helps ensure a reasonable amount
                        //of precision. The runtime renderer computes the same normalization factor to move back into texel space.
                        var decodingMultiplier = Math.Max(charData.SourceSpan.X, charData.SourceSpan.Y);
                        var encodingMultiplier = 1f / decodingMultiplier;
                        byte ToEncodedDistance(float distance)
                        {
                            return (byte)(255 * Math.Min(1, encodingMultiplier * distance));
                        }
                        //For each texel within the allocated region that has a nonzero alpha, use a breadth first traversal to mark every neighbor with a distance.
                        //If a neighbor already has a distance which is smaller, don't overwrite it.
                        //Note: this memory allocation could be optimized quite a bit; we recreate a queue for every character. It won't matter that much, though.
                        var min = new Int2((int)charData.SourceMinimum.X - padding, (int)charData.SourceMinimum.Y - padding);
                        var max = new Int2((int)(charData.SourceMinimum.X + charData.SourceSpan.X) + padding, (int)(charData.SourceMinimum.Y + charData.SourceSpan.Y) + padding);
                        var toVisit = new Queue<Int2>((int)((charData.SourceSpan.X + padding * 2) * (charData.SourceSpan.Y + padding * 2)));
                        void TryEnqueue(ref Int2 neighbor, ref Int2 origin, float baseDistance)
                        {
                            if (neighbor.X >= min.X && neighbor.X < max.X &&
                                neighbor.Y >= min.Y && neighbor.Y < max.Y)
                            {
                                //The neighbor is within the character's texel region. Is this origin closer than any previous one for this texel?
                                var xOffset = neighbor.X - origin.X;
                                var yOffset = neighbor.Y - origin.Y;
                                var candidateDistance = (float)Math.Sqrt(xOffset * xOffset + yOffset * yOffset) + baseDistance;
                                var encodedCandidateDistance = ToEncodedDistance(candidateDistance);
                                var neighborIndex = atlas.GetByteOffsetForMip0(neighbor.X, neighbor.Y);
                                //Note that this condition stops cycles. Visiting the same node from the same origin cannot produce a lower distance on later visits.
                                //It also makes the BFS early out extremely frequently- the number of traversals required for each nonzero alpha texel will decrease significantly
                                //as more are visited.
                                if (encodedCandidateDistance < atlasData[neighborIndex]) 
                                {
                                    //It's closer. 
                                    atlasData[neighborIndex] = encodedCandidateDistance;
                                    //Since this was closer, the neighbor's neighbors might be too.
                                    toVisit.Enqueue(neighbor);
                                }
                            }
                        }

                        for (int rowIndex = min.Y; rowIndex < max.Y; ++rowIndex)
                        {
                            var rowOffset = atlas.GetRowByteOffsetFromMipStart(0, rowIndex); //The alphas and distances textures have the same dimensions on mip0, so sharing this is safe.
                            var distancesRow = atlasData + rowOffset;
                            var alphasRow = alphaData + rowOffset;
                            for (int columnIndex = min.X; columnIndex < max.X; ++columnIndex)
                            {
                                if (alphasRow[columnIndex] > 0)
                                {
                                    //Nonzero alpha.
                                    //Interpret the partial alpha as a distance. (0, sqrt(2)/2) to (255, 0).
                                    //Any point that happens to have a shorter path will have this base distance overridden. It will only affect texels whose closest path 
                                    //goes through this texel.
                                    var baseDistance = 0.70710678118f - alphasRow[columnIndex] * (0.70710678118f / 255f);
                                    var encodedBaseDistance = ToEncodedDistance(baseDistance);
                                    if (distancesRow[columnIndex] > encodedBaseDistance)
                                    {
                                        //The existing distance is larger than the base distance, so it's possible to improve.
                                        distancesRow[columnIndex] = ToEncodedDistance(baseDistance);
                                        //Begin the BFS. 
                                        var origin = new Int2 { X = columnIndex, Y = rowIndex };
                                        toVisit.Enqueue(origin); //Little bit of redundant work. Shrug.
                                        while(toVisit.Count > 0)
                                        {
                                            var visited = toVisit.Dequeue();
                                            var neighbor00 = new Int2 { X = visited.X - 1, Y = visited.Y - 1 };
                                            var neighbor01 = new Int2 { X = visited.X - 1, Y = visited.Y };
                                            var neighbor02 = new Int2 { X = visited.X - 1, Y = visited.Y + 1 };
                                            var neighbor10 = new Int2 { X = visited.X, Y = visited.Y - 1 };
                                            var neighbor12 = new Int2 { X = visited.X, Y = visited.Y + 1 };
                                            var neighbor20 = new Int2 { X = visited.X + 1, Y = visited.Y - 1 };
                                            var neighbor21 = new Int2 { X = visited.X + 1, Y = visited.Y };
                                            var neighbor22 = new Int2 { X = visited.X + 1, Y = visited.Y + 1 };
                                            TryEnqueue(ref neighbor00, ref origin, baseDistance);
                                            TryEnqueue(ref neighbor01, ref origin, baseDistance);
                                            TryEnqueue(ref neighbor02, ref origin, baseDistance);
                                            TryEnqueue(ref neighbor10, ref origin, baseDistance);
                                            TryEnqueue(ref neighbor12, ref origin, baseDistance);
                                            TryEnqueue(ref neighbor20, ref origin, baseDistance);
                                            TryEnqueue(ref neighbor21, ref origin, baseDistance);
                                            TryEnqueue(ref neighbor22, ref origin, baseDistance);
                                        }
                                    }
                                }
                            }
                        }                        
                    });
                    atlas.Unpin();
                    rasterizedAlphas.Unpin();

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
