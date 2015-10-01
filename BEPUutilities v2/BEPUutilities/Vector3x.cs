using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities
{
    /// <summary>
    /// Extra functionality for the Vector3 struct.
    /// </summary>
    /// <remarks>Hopefully, all of this should eventually go away as the System.Numerics.Vectors improves.</remarks>
    public struct Vector3x
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(ref Vector3 a, ref Vector3 b, out Vector3 result)
        {
            var preresult = new Vector3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
            result = preresult;
        }
    }
}
