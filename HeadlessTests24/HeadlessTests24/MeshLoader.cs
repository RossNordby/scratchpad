using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using ObjLoader.Loader.Loaders;
using System.Numerics;

namespace HeadlessTests24
{
    public static class MeshLoader
    {
        class MaterialStubLoader : IMaterialStreamProvider
        {
            public Stream? Open(string materialFilePath)
            {
                return null;
            }
        }

        public unsafe static Mesh LoadMesh(Stream dataStream, Vector3 scale, BufferPool pool)
        {
            var objFile = new ObjLoaderFactory().Create(new MaterialStubLoader()).Load(dataStream);
            var triangles = new List<Triangle>();
            for (int i = 0; i < objFile.Groups.Count; ++i)
            {
                var group = objFile.Groups[i];
                for (int j = 0; j < group.Faces.Count; ++j)
                {
                    var face = group.Faces[j];
                    var a = objFile.Vertices[face[0].VertexIndex - 1];
                    for (int k = 1; k < face.Count - 1; ++k)
                    {
                        var b = objFile.Vertices[face[k].VertexIndex - 1];
                        var c = objFile.Vertices[face[k + 1].VertexIndex - 1];
                        triangles.Add(new Triangle
                        {
                            A = new Vector3(a.X, a.Y, a.Z),
                            B = new Vector3(b.X, b.Y, b.Z),
                            C = new Vector3(c.X, c.Y, c.Z)
                        });
                    }
                }
            }
            //This is a bit poopy. that's ok.
            pool.Take<Triangle>(triangles.Count, out var trianglesBuffer);
            for (int i = 0; i < triangles.Count; ++i)
            {
                trianglesBuffer[i] = triangles[i];

            }
            return new Mesh(trianglesBuffer, scale, pool);
        }
    }
}
