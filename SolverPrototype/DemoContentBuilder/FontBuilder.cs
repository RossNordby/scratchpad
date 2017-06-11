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

        //Text below 128 / 2^(5-1) = 8 pixels is not going to be terribly common, so generating more mips for it has pretty low value.
        //Technically, by outputting at a really high resolution and mipping it down, we miss out on low resolution hinting.
        //This could be addressed by actually rasterizing glyphs at each mip resolution and redoing the distance calculation.
        //But in practice, we'll probably just be using larger glyphs.
        private const int FontSizeInPixels = 128;
        private const int MipLevels = 5;
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
                    var packer = new FontPacker(AtlasWidth, MipLevels, padding, characterSet.Length);
                    for (int i = 0; i < sortedCharacterSet.Length; ++i)
                    {
                        //The packer class handles the placement logic and sets the SourceMinimum in the character data.
                        //So, after it is added to the intervals set, we can add the character-data pair to the font's dictionary.
                        packer.Add(ref sortedCharacterData[i]);
                        characters.Add(sortedCharacterSet[i], sortedCharacterData[i]);
                    }

                    //Now that every glyph has been positioned within the sheet, we can actually rasterize the glyph alphas into a bitmap proto-atlas.
                    //We're building the rasterized set sequentially first so we don't have to worry about threading issues in the underlying library.
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
                                ref rasterizedAlphas.Data[rasterizedAlphas.GetRowOffsetFromMipStart(0, glyphRow + location.Y) + location.X],
                                ref glyphBuffer[glyphRow * glyphWidth], (uint)glyphWidth);
                        }
                    }

                    //Preallocate memory for full single precision float version of the atlas. This will be used as scratch memory (admittedly, more than is necessary)
                    //which will be encoded into the final single byte representation after the mips are calculated. The full precision stage makes the mips a little more accurate.
                    var preciseAtlas = new Texture2DContent(AtlasWidth, packer.Height, MipLevels, 4);
                    var atlas = new Texture2DContent(AtlasWidth, packer.Height, MipLevels, 1);
                    //Compute the distances for every character-covered texel in the atlas.
                    var atlasData = atlas.Pin();
                    var preciseData = (float*)preciseAtlas.Pin();
                    var alphaData = rasterizedAlphas.Pin();
                    Parallel.For(0, sortedCharacterData.Length, i =>
                    {
                        //Note that the padding around characters should also have its distances filled in. That way, the less detailed mips can pull from useful data.
                        ref var charData = ref sortedCharacterData[i];

                        var min = new Int2((int)charData.SourceMinimum.X - padding, (int)charData.SourceMinimum.Y - padding);
                        var max = new Int2((int)(charData.SourceMinimum.X + charData.SourceSpan.X) + padding, (int)(charData.SourceMinimum.Y + charData.SourceSpan.Y) + padding);
                        //Initialize every character texel to max distance. The following BFS only ever reduces distances, so it has to start high.
                        var maxDistance = Math.Max(AtlasWidth, packer.Height);
                        for (int rowIndex = min.Y; rowIndex < max.Y; ++rowIndex)
                        {
                            var rowOffset = preciseAtlas.GetRowOffsetFromMipStart(0, rowIndex);
                            var distancesRow = preciseData + rowOffset;
                            for (int columnIndex = min.X; columnIndex < max.X; ++columnIndex)
                            {
                                distancesRow[columnIndex] = maxDistance;
                            }
                        }

                        //For each texel within the allocated region that has a nonzero alpha, use a breadth first traversal to mark every neighbor with a distance.
                        //If a neighbor already has a distance which is smaller, don't overwrite it.
                        //Note: this memory allocation could be optimized quite a bit; we recreate a queue for every character. It won't matter that much, though.
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
                                var neighborIndex = preciseAtlas.GetOffsetForMip0(neighbor.X, neighbor.Y);
                                //Note that this condition stops cycles. Visiting the same node from the same origin cannot produce a lower distance on later visits.
                                //It also makes the BFS early out extremely frequently- the number of traversals required for each nonzero alpha texel will decrease significantly
                                //as more are visited.
                                if (candidateDistance < preciseData[neighborIndex])
                                {
                                    //It's closer. 
                                    preciseData[neighborIndex] = candidateDistance;
                                    //Since this was closer, the neighbor's neighbors might be too.
                                    toVisit.Enqueue(neighbor);
                                }
                            }
                        }

                        for (int rowIndex = min.Y; rowIndex < max.Y; ++rowIndex)
                        {
                            var rowOffset = preciseAtlas.GetRowOffsetFromMipStart(0, rowIndex); //The alphas and distances textures have the same dimensions on mip0, so sharing this is safe.
                            var distancesRow = preciseData + rowOffset;
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
                                    if (distancesRow[columnIndex] > baseDistance)
                                    {
                                        //The existing distance is larger than the base distance, so it's possible to improve.
                                        distancesRow[columnIndex] = baseDistance;
                                        //Begin the BFS. 
                                        var origin = new Int2 { X = columnIndex, Y = rowIndex };
                                        toVisit.Enqueue(origin); //Little bit of redundant work. Shrug.
                                        while (toVisit.Count > 0)
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
                        //Build the mips. We already have all the data in cache on this core; 256KiB L2 can easily hold the processing context of a 128x128 glyph.
                        //(Though worrying about performance in the content builder too much is pretty silly. We aren't going to be building fonts often.)
                        //Note that we aligned and padded each glyph during packing. For a given texel in mip(n), the four parent texels in mip(n-1) can be safely sampled.
                        for (int mipLevel = 1; mipLevel < preciseAtlas.MipLevels; ++mipLevel)
                        {
                            var mipMin = new Int2(min.X >> mipLevel, min.Y >> mipLevel);
                            var mipMax = new Int2(max.X >> mipLevel, max.Y >> mipLevel);

                            //Yes, these do some redundant calculations, but no it doesn't matter.
                            var parentMipStart = preciseData + preciseAtlas.GetMipStartIndex(mipLevel - 1);
                            var parentMipRowPitch = preciseAtlas.GetRowPitch(mipLevel - 1);
                            var mipStart = preciseData + preciseAtlas.GetMipStartIndex(mipLevel);
                            var mipRowPitch = preciseAtlas.GetRowPitch(mipLevel);
                            for (int mipRowIndex = mipMin.Y; mipRowIndex < mipMax.Y; ++mipRowIndex)
                            {
                                var mipRow = mipStart + mipRowIndex * mipRowPitch;
                                var parentRowIndex = mipRowIndex << 1;
                                var parentMipRow0 = parentMipStart + parentRowIndex * parentMipRowPitch;
                                var parentMipRow1 = parentMipStart + (parentRowIndex + 1) * parentMipRowPitch;
                                for (int mipColumnIndex = mipMin.X; mipColumnIndex < mipMax.X; ++mipColumnIndex)
                                {
                                    var parentMipColumnIndex0 = mipColumnIndex << 1;
                                    var parentMipColumnIndex1 = parentMipColumnIndex0 + 1;
                                    mipRow[mipColumnIndex] = 0.25f * (
                                        parentMipRow0[parentMipColumnIndex0] + parentMipRow0[parentMipColumnIndex1] +
                                        parentMipRow1[parentMipColumnIndex0] + parentMipRow1[parentMipColumnIndex1]);
                                }

                            }
                        }

                        //Now that all mips have been filled, bake the data into the final single byte encoding.
                        //Note that the conversion from distance to texels involves a normalization by the character's span. That helps ensure a reasonable amount
                        //of precision. The runtime renderer computes the same normalization factor to move back into texel space.
                        var encodingMultiplier = 1f / Math.Max(charData.SourceSpan.X, charData.SourceSpan.Y);
                        for (int mipLevel = 0; mipLevel < atlas.MipLevels; ++mipLevel)
                        {
                            var mipMin = new Int2(min.X >> mipLevel, min.Y >> mipLevel);
                            var mipMax = new Int2(max.X >> mipLevel, max.Y >> mipLevel);

                            var encodedStart = atlasData + atlas.GetMipStartIndex(mipLevel);
                            var preciseStart = preciseData + preciseAtlas.GetMipStartIndex(mipLevel);
                            var rowPitch = atlas.GetRowPitch(mipLevel);
                            for (int rowIndex = mipMin.Y; rowIndex < mipMax.Y; ++rowIndex)
                            {
                                var preciseRow = preciseStart + rowIndex * rowPitch;
                                var encodedRow = encodedStart + rowIndex * rowPitch;
                                for (int columnIndex = mipMin.X; columnIndex < mipMax.X; ++columnIndex)
                                {
                                    encodedRow[columnIndex] = (byte)(255 * Math.Min(1, encodingMultiplier * preciseRow[columnIndex]));
                                }

                            }
                        }
                    });
                    atlas.Unpin();
                    preciseAtlas.Unpin();
                    rasterizedAlphas.Unpin();


                    //Build the kerning table.
                    var kerning = ComputeKerningTable(face, characterSet);

                    return new FontContent(atlas, face.FamilyName, 1f / FontSizeInPixels, characters, kerning);
                }
            }
        }




    }
}
