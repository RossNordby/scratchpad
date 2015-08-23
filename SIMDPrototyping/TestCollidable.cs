using BEPUphysics.BroadPhaseSystems;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping.Trees
{
    public class TestCollidable : IBounded
    {
        public Vector3 Position;
        public Vector3 HalfSize;
        public Vector3 Velocity;

        public void GetBoundingBox(out BoundingBox box)
        {
            box.Min = Position - HalfSize;
            box.Max = Position + HalfSize;
        }
    }

    public class TestCollidableBEPU : IBoundingBoxOwner
    {

        public BEPUutilities.BoundingBox BoundingBox
        {
            get; set;
        }
    }
}

