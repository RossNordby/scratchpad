using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace SIMDPrototyping
{
    [StructLayout(LayoutKind.Explicit, Size = 160)]
    public struct RigidBody
    {
        [FieldOffset(0)]
        public Vector3 Position;
        [FieldOffset(16)]
        public Matrix3x3 Orientation;
        [FieldOffset(64)]
        public Vector3 LinearVelocity;
        [FieldOffset(80)]
        public Vector3 AngularVelocity;
        [FieldOffset(96)]
        public Matrix3x3 InertiaTensorInverse;
        [FieldOffset(144)]
        public float InverseMass;
    }
}
