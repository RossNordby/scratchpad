

using System.Numerics;
using System.Runtime.CompilerServices;

namespace BEPUutilities2
{
    ///<summary>
    /// A transformation composed of a linear transformation and a translation.
    ///</summary>
    public struct AffineTransform
    {
        ///<summary>
        /// Translation in the affine transform.
        ///</summary>
        public Vector3 Translation;
        /// <summary>
        /// Linear transform in the affine transform.
        /// </summary>
        public Matrix3x3 LinearTransform;



        ///<summary>
        /// Gets the identity affine transform.
        ///</summary>
        public static AffineTransform Identity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                var t = new AffineTransform { LinearTransform = Matrix3x3.Identity };
                return t;
            }
        }

        ///<summary>
        /// Transforms a vector by an affine transform.
        ///</summary>
        ///<param name="position">Position to transform.</param>
        ///<param name="transform">Transform to apply.</param>
        ///<param name="transformed">Transformed position.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(ref Vector3 position, ref AffineTransform transform, out Vector3 transformed)
        {
            Matrix3x3.Transform(ref position, ref transform.LinearTransform, out transformed);
            transformed += transform.Translation;
        }

        ///<summary>
        /// Inverts an affine transform.
        ///</summary>
        ///<param name="transform">Transform to invert.</param>
        /// <param name="inverse">Inverse of the transform.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(ref AffineTransform transform, out AffineTransform inverse)
        {
            Matrix3x3.Invert(ref transform.LinearTransform, out inverse.LinearTransform);
            Matrix3x3.Transform(ref transform.Translation, ref inverse.LinearTransform, out inverse.Translation);
            inverse.Translation = -inverse.Translation;
        }

        ///<summary>
        /// Inverts a rigid transform.
        ///</summary>
        ///<param name="transform">Transform to invert.</param>
        /// <param name="inverse">Inverse of the transform.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InvertRigid(ref AffineTransform transform, out AffineTransform inverse)
        {
            Matrix3x3.Transpose(ref transform.LinearTransform, out inverse.LinearTransform);
            Matrix3x3.Transform(ref transform.Translation, ref inverse.LinearTransform, out inverse.Translation);
            inverse.Translation = -inverse.Translation;
        }

        /// <summary>
        /// Multiplies a transform by another transform.
        /// </summary>
        /// <param name="a">First transform.</param>
        /// <param name="b">Second transform.</param>
        /// <param name="transform">Combined transform.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(ref AffineTransform a, ref AffineTransform b, out AffineTransform transform)
        {
            Vector3 translation;
            Matrix3x3.Transform(ref a.Translation, ref b.LinearTransform, out translation);
            transform.Translation = b.Translation + translation;
            Matrix3x3.Multiply(ref a.LinearTransform, ref b.LinearTransform, out transform.LinearTransform);
        }



    }
}