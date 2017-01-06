using System.Numerics;
using System.Runtime.CompilerServices;

namespace SolverPrototype
{
    public struct Matrix3x3SOA
    {
        public float[] M11;
        public float[] M12;
        public float[] M13;
        public float[] M21;
        public float[] M22;
        public float[] M23;
        public float[] M31;
        public float[] M32;
        public float[] M33;
    }

    public struct Matrix3x3Wide
    {
        public Vector<float> M11;
        public Vector<float> M12;
        public Vector<float> M13;
        public Vector<float> M21;
        public Vector<float> M22;
        public Vector<float> M23;
        public Vector<float> M31;
        public Vector<float> M32;
        public Vector<float> M33;

        public Matrix3x3Wide(ref Matrix3x3SOA source, int index)
        {
            M11 = new Vector<float>(source.M11, index);
            M12 = new Vector<float>(source.M12, index);
            M13 = new Vector<float>(source.M13, index);
            M21 = new Vector<float>(source.M21, index);
            M22 = new Vector<float>(source.M22, index);
            M23 = new Vector<float>(source.M23, index);
            M31 = new Vector<float>(source.M31, index);
            M32 = new Vector<float>(source.M32, index);
            M33 = new Vector<float>(source.M33, index);
        }

    }
    public struct Vector3Wide
    {
        public Vector<float> X;
        public Vector<float> Y;
        public Vector<float> Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(ref Vector3Wide a, ref Vector3Wide b, out Vector3Wide result)
        {
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3Wide operator +(Vector3Wide a, Vector3Wide b)
        {
            Vector3Wide result;
            result.X = a.X + b.X;
            result.Y = a.Y + b.Y;
            result.Z = a.Z + b.Z;
            return result;
        }
    }


    public struct Matrix3x3SOA2
    {
        public Vector<float>[] M11;
        public Vector<float>[] M12;
        public Vector<float>[] M13;
        public Vector<float>[] M21;
        public Vector<float>[] M22;
        public Vector<float>[] M23;
        public Vector<float>[] M31;
        public Vector<float>[] M32;
        public Vector<float>[] M33;
    }

    public struct InequalityConstraint1DOF
    {
        public void Solve()
        {

        }
    }
}
