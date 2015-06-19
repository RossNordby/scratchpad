using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    public struct RigidBody
    {
        public Vector3 Position;
        public Matrix3x3 Orientation;
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;
        public float InverseMass;
        public Matrix3x3 InertiaTensorInverse;
    }
}
