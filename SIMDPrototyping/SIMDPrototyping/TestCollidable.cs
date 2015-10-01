using BEPUphysics.BroadPhaseEntries;
using BEPUphysics.BroadPhaseSystems;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using BEPUphysics.CollisionShapes.ConvexShapes;

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

    public class TestCollidableBEPU : BroadPhaseEntry
    {
        public BEPUutilities.Vector3 Position;
        public BEPUutilities.Vector3 HalfSize;
        public BEPUutilities.Vector3 Velocity;

        protected override bool IsActive
        {
            get
            {
                return true;
            }
        }

        public override bool ConvexCast(ConvexShape castShape, ref BEPUutilities.RigidTransform startingTransform, ref BEPUutilities.Vector3 sweep, out BEPUutilities.RayHit hit)
        {
            throw new NotImplementedException();
        }

        public override bool RayCast(BEPUutilities.Ray ray, float maximumLength, out BEPUutilities.RayHit rayHit)
        {
            throw new NotImplementedException();
        }

        public override void UpdateBoundingBox()
        {
            BEPUutilities.Vector3.Subtract(ref Position, ref HalfSize, out boundingBox.Min);
            BEPUutilities.Vector3.Add(ref Position, ref HalfSize, out boundingBox.Max);
        }

        protected override void CollisionRulesUpdated()
        {
        }
    }
}

