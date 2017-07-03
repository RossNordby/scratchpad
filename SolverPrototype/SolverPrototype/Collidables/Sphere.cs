using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BEPUutilities2.Memory;
using System.Diagnostics;

namespace SolverPrototype.Collidables
{

    public struct Sphere : IShape
    {
        public float Radius;

        public Sphere(float radius)
        {
            Radius = radius;
        }
        
        //Note that spheres are sufficiently simple that no explicit bundle is required. A single vector<float> suffices.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref Buffer<Sphere> shapes, ref Vector<int> shapeIndices, int count, out Vector<float> radii)
        {
            ref var radiiBase = ref Unsafe.As<Vector<float>, float>(ref radii);
            ref var shapeIndicesBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            Debug.Assert(count <= Vector<float>.Count);
            for (int i = 0; i < count; ++i)
            {
                Unsafe.Add(ref radiiBase, i) = shapes[Unsafe.Add(ref shapeIndicesBase, i)].Radius;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref BodyPoses poses,
            out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
            where TShape : struct, IShape
        {
            Gather(ref Unsafe.As<Buffer<TShape>, Buffer<Sphere>>(ref shapes), ref shapeIndices, count, out var radii);

            //Spheres have perfect symmetry, so there is no need for angular expansion.
            maximumRadius = new Vector<float>();
            maximumAngularExpansion = new Vector<float>();

            //It's technically true that spheres (and only spheres) do not require orientation to be loaded and could be special cased to reduce memory traffic, but just heck no.
            //It's very likely that the orientation loaded for the sphere was already in L1 anyway due to the online batching performed during the pose integrator.
            Vector3Wide.Subtract(ref poses.Position, ref radii, out min);
            Vector3Wide.Add(ref poses.Position, ref radii, out max);
        }
    }
}
