﻿using BEPUutilities2;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;

namespace DemoRenderer.Constraints
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single line instance.
    /// </summary>
    public struct LineInstance
    {
        public Vector3 Start;
        public float Radius;
        public Vector3 End;
        public uint PackedColor;

        public LineInstance(ref Vector3 start, ref Vector3 end, float radius, ref Vector3 color)
        {
            Start = start;
            End = end;
            Radius = radius;
            PackedColor = Helpers.PackColor(ref color);
        }
    }

    public class LineRenderer : IDisposable
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
            public Vector3 CameraBackward;
        }
        ConstantsBuffer<VertexConstants> vertexConstants;
        [StructLayout(LayoutKind.Explicit)]
        struct PixelConstants
        {
            [FieldOffset(0)]
            public Vector3 CameraForward;
            [FieldOffset(16)]
            public float NearClip;
            [FieldOffset(20)]
            public float FarClip;
        }
        ConstantsBuffer<PixelConstants> pixelConstants;

        StructuredBuffer<LineInstance> instances;
        IndexBuffer indices;

        VertexShader vertexShader;
        PixelShader pixelShader;

        public LineRenderer(Device device, ShaderCache cache, int maximumInstancesPerDraw = 2048)
        {
            instances = new StructuredBuffer<LineInstance>(device, maximumInstancesPerDraw, "Line Instances");
            indices = new IndexBuffer(Helpers.GetQuadIndices(maximumInstancesPerDraw), device, "Sphere AABB Indices");

            vertexConstants = new ConstantsBuffer<VertexConstants>(device, debugName: "Sphere Renderer Vertex Constants");
            pixelConstants = new ConstantsBuffer<PixelConstants>(device, debugName: "Sphere Renderer Pixel Constants");

            vertexShader = new VertexShader(device, cache.GetShader(@"Bodies\RenderSpheres.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"Bodies\RenderSpheres.hlsl.pshader"));
        }

        public void Render(DeviceContext context, Camera camera, LineInstance[] instances, int start, int count)
        {
            var vertexConstantsData = new VertexConstants
            {
                Projection = Matrix.Transpose(camera.Projection), //compensate for the shader packing.
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                NearClip = camera.NearClip,
                CameraUp = camera.Up,
                CameraBackward = camera.Backward,
            };
            vertexConstants.Update(context, ref vertexConstantsData);
            var pixelConstantsData = new PixelConstants
            {
                CameraForward = camera.Forward,
                NearClip = camera.NearClip,
                FarClip = camera.FarClip
            };
            pixelConstants.Update(context, ref pixelConstantsData);

            //This assumes that render states have been set appropriately for opaque rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.SetIndexBuffer(indices);
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, this.instances.SRV);
            context.PixelShader.Set(pixelShader);
            context.PixelShader.SetConstantBuffer(0, pixelConstants.Buffer);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(context, instances, batchCount, start);
                context.DrawIndexed(batchCount * 36, 0, 0);
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
        ~SphereRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
