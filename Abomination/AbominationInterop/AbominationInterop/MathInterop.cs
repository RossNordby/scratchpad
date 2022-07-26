using System.Numerics;
using System.Runtime.Intrinsics;

namespace AbominationInterop;

/// <summary>
/// Vector3Wide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
public struct Vector3SIMD128
{
    public Vector128<float> X;
    public Vector128<float> Y;
    public Vector128<float> Z;
}

/// <summary>
/// Vector3Wide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
public struct Vector3SIMD256
{
    public Vector256<float> X;
    public Vector256<float> Y;
    public Vector256<float> Z;
}

/// <summary>
/// QuaternionWide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
public struct QuaternionSIMD128
{
    public Vector128<float> X;
    public Vector128<float> Y;
    public Vector128<float> Z;
}

/// <summary>
/// QuaternionWide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
public struct QuaternionSIMD256
{
    public Vector256<float> X;
    public Vector256<float> Y;
    public Vector256<float> Z;
}

/// <summary>
/// BodyInertiaWide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
public struct BodyInertiaSIMD128
{
    public Vector128<float> InverseInertiaXX;
    public Vector128<float> InverseInertiaYX;
    public Vector128<float> InverseInertiaYY;
    public Vector128<float> InverseInertiaZX;
    public Vector128<float> InverseInertiaZY;
    public Vector128<float> InverseInertiaZZ;
    public Vector128<float> InverseMass;
}

/// <summary>
/// BodyInertiaWide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
public struct BodyInertiaSIMD256
{
    public Vector256<float> InverseInertiaXX;
    public Vector256<float> InverseInertiaYX;
    public Vector256<float> InverseInertiaYY;
    public Vector256<float> InverseInertiaZX;
    public Vector256<float> InverseInertiaZY;
    public Vector256<float> InverseInertiaZZ;
    public Vector256<float> InverseMass;
}