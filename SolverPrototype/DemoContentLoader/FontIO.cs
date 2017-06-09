using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace DemoContentLoader
{
    public class FontIO
    {
        public static FontContent Load(BinaryReader reader)
        {
            var name = reader.ReadString();
            var inverseSizeInTexels = reader.ReadSingle();
            var glyphCount = reader.ReadInt32();
            var characterData = new Dictionary<char, CharacterData>();
            for (int i = 0; i < glyphCount; ++i)
            {
                var character = reader.ReadChar();
                CharacterData data;
                data.SourceMinimum.X = reader.ReadSingle();
                data.SourceMinimum.Y = reader.ReadSingle();
                data.SourceSpan.X = reader.ReadSingle();
                data.SourceSpan.Y = reader.ReadSingle();
                data.Bearing.X = reader.ReadSingle();
                data.Bearing.Y = reader.ReadSingle();
                data.Advance = reader.ReadSingle();
            }
            var kerningRelationshipCount = reader.ReadInt32();
            var kerningTable = new Dictionary<CharacterPair, float>();
            for (int i = 0; i < kerningRelationshipCount; ++i)
            {
                var a = reader.ReadChar();
                var b = reader.ReadChar();
                var amount = reader.ReadSingle();
                kerningTable.Add(new CharacterPair(a, b), amount);
            }
            var atlas = Texture2DIO.Load(reader);
            return new FontContent(atlas, name, inverseSizeInTexels, characterData, kerningTable);
        }

        public static void Save(FontContent content, BinaryWriter writer)
        {
            writer.Write(content.Name);
            writer.Write(content.InverseSizeInTexels);
            writer.Write(content.Characters.Count);
            foreach (var pair in content.Characters)
            {
                writer.Write(pair.Key);
                writer.Write(pair.Value.SourceMinimum.X);
                writer.Write(pair.Value.SourceMinimum.Y);
                writer.Write(pair.Value.SourceSpan.X);
                writer.Write(pair.Value.SourceSpan.Y);
                writer.Write(pair.Value.Bearing.X);
                writer.Write(pair.Value.Bearing.Y);
                writer.Write(pair.Value.Advance);
            }
            writer.Write(content.kerning.Count);
            foreach (var pair in content.kerning)
            {
                writer.Write(pair.Key.A);
                writer.Write(pair.Key.B);
                writer.Write(pair.Value);
            }
            Texture2DIO.Save(content.Atlas, writer);
        }
    }
}