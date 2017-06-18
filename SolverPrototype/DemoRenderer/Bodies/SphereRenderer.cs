using BEPUutilities2;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Quaternion = BEPUutilities2.Quaternion;

namespace DemoRenderer.Bodies
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single sphere instance.
    /// </summary>
    public struct SphereInstance
    {
        public Vector3 Position;
        public float Radius;
        public Quaternion Orientation;
    }

    public class SphereRenderer : IDisposable
    {
        [StructLayout(LayoutKind.Explicit)]
        struct VertexConstants
        {
            [FieldOffset(0)]
            public Matrix Projection;
            [FieldOffset(64)]
            public Vector3 CameraPosition;
            [FieldOffset(76)]
            public float NearClip;
            [FieldOffset(80)]
            public Vector3 CameraRight;
            [FieldOffset(96)]
            public Vector3 CameraUp;
            [FieldOffset(112)]
            public Vector3 CameraForward;
        }
        ConstantsBuffer<VertexConstants> vertexConstants;

        StructuredBuffer<SphereInstance> instances;
        IndexBuffer indices;

        VertexShader vertexShader;
        PixelShader pixelShader;

        public SphereRenderer(Device device, ShaderCache cache, int maximumInstancesPerDraw = 2048)
        {
            instances = new StructuredBuffer<SphereInstance>(device, maximumInstancesPerDraw, "Sphere Instances");
            //Build a AABB mesh's index buffer, repeated. A redundant index buffer tends to be faster than instancing tiny models. Impact varies between hardware.
            var indexData = new uint[maximumInstancesPerDraw * 6 * 6];
            uint baseVertex = 0;
            for (int glyphIndexStart = 0; glyphIndexStart < indexData.Length; glyphIndexStart += 6)
            {
                //+X
                indexData[glyphIndexStart + 0] = baseVertex + 1;
                indexData[glyphIndexStart + 1] = baseVertex + 3;
                indexData[glyphIndexStart + 2] = baseVertex + 7;
                indexData[glyphIndexStart + 3] = baseVertex + 7;
                indexData[glyphIndexStart + 4] = baseVertex + 5;
                indexData[glyphIndexStart + 5] = baseVertex + 1;
                //-X
                indexData[glyphIndexStart + 6] = baseVertex + 4;
                indexData[glyphIndexStart + 7] = baseVertex + 6;
                indexData[glyphIndexStart + 8] = baseVertex + 2;
                indexData[glyphIndexStart + 9] = baseVertex + 2;
                indexData[glyphIndexStart + 10] = baseVertex + 0;
                indexData[glyphIndexStart + 11] = baseVertex + 4;
                //+Y
                indexData[glyphIndexStart + 12] = baseVertex + 4;
                indexData[glyphIndexStart + 13] = baseVertex + 0;
                indexData[glyphIndexStart + 14] = baseVertex + 1;
                indexData[glyphIndexStart + 15] = baseVertex + 1;
                indexData[glyphIndexStart + 16] = baseVertex + 5;
                indexData[glyphIndexStart + 17] = baseVertex + 4;
                //-Y
                indexData[glyphIndexStart + 18] = baseVertex + 7;
                indexData[glyphIndexStart + 19] = baseVertex + 3;
                indexData[glyphIndexStart + 20] = baseVertex + 2;
                indexData[glyphIndexStart + 21] = baseVertex + 2;
                indexData[glyphIndexStart + 22] = baseVertex + 6;
                indexData[glyphIndexStart + 23] = baseVertex + 7;
                //+Z
                indexData[glyphIndexStart + 24] = baseVertex + 0;
                indexData[glyphIndexStart + 25] = baseVertex + 2;
                indexData[glyphIndexStart + 26] = baseVertex + 3;
                indexData[glyphIndexStart + 27] = baseVertex + 3;
                indexData[glyphIndexStart + 28] = baseVertex + 1;
                indexData[glyphIndexStart + 29] = baseVertex + 0;
                //-Z
                indexData[glyphIndexStart + 30] = baseVertex + 5;
                indexData[glyphIndexStart + 31] = baseVertex + 7;
                indexData[glyphIndexStart + 32] = baseVertex + 6;
                indexData[glyphIndexStart + 33] = baseVertex + 6;
                indexData[glyphIndexStart + 34] = baseVertex + 4;
                indexData[glyphIndexStart + 35] = baseVertex + 5;
                baseVertex += 4;
            }
            indices = new IndexBuffer(Helpers.GetScreenQuadIndices(maximumInstancesPerDraw), device, "Sphere AABB Indices");

            vertexConstants = new ConstantsBuffer<VertexConstants>(device, debugName: "Sphere Renderer Vertex Constants");

            vertexShader = new VertexShader(device, cache.GetShader(@"Bodies\RenderSpheres.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"Bodies\RenderSpheres.hlsl.pshader"));
        }

        public void Render(DeviceContext context, Camera camera, SphereInstance[] instances, int start, int count)
        {
            //This assumes that render states have been set appropriately for opaque rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.SetIndexBuffer(indices);
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, this.instances.SRV);
            var vertexConstantsData = new VertexConstants
            {
                Projection = Matrix.Transpose(camera.Projection), //compensate for the shader packing.
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                NearClip = camera.NearClip,
                CameraUp = camera.Up,
                CameraForward = camera.Forward,
            };
            vertexConstants.Update(context, ref vertexConstantsData);
            context.PixelShader.Set(pixelShader);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(context, instances, batchCount, start);
                context.DrawIndexed(batchCount * 6, 0, 0);
                count -= batchCount;
                start += batchCount;
            }
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                vertexShader.Dispose();
                pixelShader.Dispose();
                instances.Dispose();
                indices.Dispose();
                vertexConstants.Dispose();
            }
        }

#if DEBUG
        ~UILineRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
