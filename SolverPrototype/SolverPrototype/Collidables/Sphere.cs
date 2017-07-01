using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BEPUutilities2.Memory;

namespace SolverPrototype.Collidables
{
    public struct Sphere : IShape
    {
        public float Radius;

        public Sphere(float radius)
        {
            Radius = radius;
        }

        public float MaximumRadius
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return Radius; }
        }
        
    }

    public struct SphereBundle : IShapeBundle
    {
        public Vector<float> Radii;

        public SphereBundle(ShapeBatch<Sphere> spheres, Vector<int> shapeIndices)
        {
            ref var radiiStart = ref Unsafe.As<Vector<float>, float>(ref Radii);
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                Unsafe.Add(ref radiiStart, i) = spheres[shapeIndices[i]].Radius;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeBoundingBoxes(ref BodyPoses poses, out Vector3Wide min, out Vector3Wide max)
        {
            //It's technically true that spheres (and only spheres) do not require orientation to be loaded and could be special cased to reduce memory traffic, but just heck no.
            //It's very likely that the orientation loaded for the sphere was already in L1 anyway due to the online batching performed during the pose integrator.
            Vector3Wide.Subtract(ref poses.Position, ref Radii, out min);
            Vector3Wide.Add(ref poses.Position, ref Radii, out max);

        }

    }
}
