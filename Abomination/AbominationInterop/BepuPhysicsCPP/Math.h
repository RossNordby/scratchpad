#pragma once

#include <stdint.h>

struct Vector3
{
	float X;
	float Y;
	float Z;
};
struct Quaternion
{
	float X;
	float Y;
	float Z;
	float W;
};


struct Vector128F
{
	float V0;
	float V1;
	float V2;
	float V3;
};
struct Vector256F
{
	float V0;
	float V1;
	float V2;
	float V3;
	float V4;
	float V5;
	float V6;
	float V7;
};
struct Vector128I
{
	int32_t V0;
	int32_t V1;
	int32_t V2;
	int32_t V3;
};
struct Vector256I
{
	int32_t V0;
	int32_t V1;
	int32_t V2;
	int32_t V3;
	int32_t V4;
	int32_t V5;
	int32_t V6;
	int32_t V7;
};

/// <summary>
/// Represents a rigid transformation.
/// </summary>
struct RigidPose
{
	/// <summary>
	/// Orientation of the pose.
	/// </summary>
	Quaternion Orientation;
	/// <summary>
	/// Position of the pose.
	/// </summary>
	Vector3 Position;
	int32_t Pad;
};

/// <summary>
/// Lower left triangle (including diagonal) of a symmetric 3x3 matrix.
/// </summary>
struct alignas(float) Symmetric3x3
{
	/// <summary>
	/// First row, first column of the matrix.
	/// </summary>
	float XX;
	/// <summary>
	/// Second row, first column of the matrix.
	/// </summary>
	float YX;
	/// <summary>
	/// Second row, second column of the matrix.
	/// </summary>
	float YY;
	/// <summary>
	/// Third row, first column of the matrix.
	/// </summary>
	float ZX;
	/// <summary>
	/// Third row, second column of the matrix.
	/// </summary>
	float ZY;
	/// <summary>
	/// Third row, third column of the matrix.
	/// </summary>
	float ZZ;
};


/// <summary>
/// Vector3Wide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
struct Vector3SIMD128
{
	Vector128F X;
	Vector128F Y;
	Vector128F Z;
};

/// <summary>
/// Vector3Wide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
struct Vector3SIMD256
{
	Vector256F X;
	Vector256F Y;
	Vector256F Z;
};

/// <summary>
/// QuaternionWide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
struct QuaternionSIMD128
{
	Vector128F X;
	Vector128F Y;
	Vector128F Z;
};

/// <summary>
/// QuaternionWide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
struct QuaternionSIMD256
{
	Vector256F X;
	Vector256F Y;
	Vector256F Z;
};

/// <summary>
/// BodyInertiaWide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
struct BodyInertiaSIMD128
{
	Vector128F InverseInertiaXX;
	Vector128F InverseInertiaYX;
	Vector128F InverseInertiaYY;
	Vector128F InverseInertiaZX;
	Vector128F InverseInertiaZY;
	Vector128F InverseInertiaZZ;
	Vector128F InverseMass;
};

/// <summary>
/// BodyInertiaWide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
struct BodyInertiaSIMD256
{
	Vector256F InverseInertiaXX;
	Vector256F InverseInertiaYX;
	Vector256F InverseInertiaYY;
	Vector256F InverseInertiaZX;
	Vector256F InverseInertiaZY;
	Vector256F InverseInertiaZZ;
	Vector256F InverseMass;
};

/// <summary>
/// BodyVelocityWide interop type used when <see cref="Vector{float}"/> is 128 bits wide.
/// </summary>
struct BodyVelocitySIMD128
{
	Vector3SIMD128 Linear;
	Vector3SIMD128 Angular;
};

/// <summary>
/// BodyVelocityWide interop type used when <see cref="Vector{float}"/> is 256 bits wide.
/// </summary>
struct BodyVelocitySIMD256
{
	Vector3SIMD256 Linear;
	Vector3SIMD256 Angular;
};
