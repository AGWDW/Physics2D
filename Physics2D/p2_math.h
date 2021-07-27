#pragma once
#include <math.h>
#include "p2_settings.h"

inline bool b2IsValid(float x)
{
	return isfinite(x);
}

struct p2Vec2 {
	/// Default constructor does nothing (for performance).
	p2Vec2() : x(0), y(0) { }

	/// Construct using coordinates.
	p2Vec2(float xIn, float yIn) : x(xIn), y(yIn) { }

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float x_, float y_) { x = x_; y = y_; }

	/// Negate this vector.
	p2Vec2 operator -() const { p2Vec2 v; v.Set(-x, -y); return v; }

	/// Read from and indexed element.
	float operator () (int i) const
	{
		return (&x)[i];
	}

	/// Write to an indexed element.
	float& operator () (int i)
	{
		return (&x)[i];
	}

	/// Add a vector to this vector.
	void operator += (const p2Vec2& v)
	{
		x += v.x; y += v.y;
	}

	/// Subtract a vector from this vector.
	void operator -= (const p2Vec2& v)
	{
		x -= v.x; y -= v.y;
	}

	/// Multiply this vector by a scalar.
	void operator *= (float a)
	{
		x *= a; y *= a;
	}

	/// Get the length of this vector (the norm).
	float Length() const
	{
		return sqrtf(x * x + y * y);
	}

	/// Get the length squared. For performance, use this instead of
	/// p2Vec2::Length (if possible).
	float LengthSquared() const
	{
		return x * x + y * y;
	}

	/// Convert this vector into a unit vector. Returns the length.
	float Normalize()
	{
		float length = Length();
		if (length < p2_Epsilon)
		{
			return 0.0f;
		}
		float invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;

		return length;
	}

	/// Does this vector contain finite coordinates?
	bool IsValid() const
	{
		return b2IsValid(x) && b2IsValid(y);
	}

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	p2Vec2 Skew() const
	{
		return p2Vec2(-y, x);
	}

	float x, y;
};

struct p2Vec3 {
	/// Default constructor does nothing (for performance).
	p2Vec3() : x(0), y(0), z(0) { }

	/// Construct using coordinates.
	p2Vec3(float xIn, float yIn, float zIn) : x(xIn), y(yIn), z(zIn) { }

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }

	/// Negate this vector.
	p2Vec3 operator -() const { p2Vec3 v; v.Set(-x, -y, -z); return v; }

	/// Add a vector to this vector.
	void operator += (const p2Vec3& v)
	{
		x += v.x; y += v.y; z += v.z;
	}

	/// Subtract a vector from this vector.
	void operator -= (const p2Vec3& v)
	{
		x -= v.x; y -= v.y; z -= v.z;
	}

	/// Multiply this vector by a scalar.
	void operator *= (float s)
	{
		x *= s; y *= s; z *= s;
	}

	float x, y, z;
};

/// <summary>
/// Angle in radias doesnt store the angle but rarther sin and cos results
/// </summary>
struct p2Rot {
	p2Rot() : s(0), c (0) { }

	p2Rot(float angle) : p2Rot() {
		s = sinf(angle);
		c = cosf(angle);
	}

	void Set(float angle) {
		s = sinf(angle);
		c = cosf(angle);
	}
	/// <summary>
	/// Gets the angle in radians
	/// </summary>
	float GetAngle() {
		return atan2(s, c);
	}

	p2Vec2 GetXAxis() {
		return p2Vec2(c, s);
	}

	p2Vec2 GetYAxis() {
		return p2Vec2(-s, c);
	}

	void SetIdentify() {
		s = 0;
		c = 1;
	}
	float s, c;
};

struct p2Transform {
	p2Vec2 position;
	p2Rot rotation;

	p2Transform() : position(), rotation() { }

	p2Transform(p2Vec2 pos, p2Rot rot) : position(pos), rotation(rot) { }

	/// Set this to the identity transform.
	void SetIdentity() {
		position.SetZero();
		rotation.SetIdentify();
	}

	/// Set this based on the position and angle.
	void Set(const p2Vec2& pos, float angle)
	{
		position = position;
		rotation.Set(angle);
	}

};

/// Adds 2 vectors toggerther
inline p2Vec2 operator +(const p2Vec2& a, const p2Vec2& b) {
	return p2Vec2(a.x + b.x, a.y + b.y);
}

/// Subtracts 2 vectors toggerther
inline p2Vec2 operator -(const p2Vec2& a, const p2Vec2& b) {
	return p2Vec2(a.x - b.x, a.y - b.y);
}

/// Scalar multliplication
inline p2Vec2 operator *(const float& a, const p2Vec2& b) {
	return p2Vec2(a + b.x, a + b.y);
}

/// Scalar multliplication
inline p2Vec2 operator *(const p2Vec2& a, const float& b) {
	return p2Vec2(a.x + b, a.y + b);
}

inline p2Vec2 p2Min(p2Vec2 a, p2Vec2 b) {
	return p2Vec2(fminf(a.x, b.x), fminf(a.y, b.y));
}

inline p2Vec2 p2Max(p2Vec2 a, p2Vec2 b) {
	return p2Vec2(fmaxf(a.x, b.x), fmaxf(a.y, b.y));
}

inline p2Vec2 b2Mul(const p2Transform& T, const p2Vec2& v)
{
	float x = (T.rotation.c * v.x - T.rotation.s * v.y) + T.position.x;
	float y = (T.rotation.s * v.x + T.rotation.c * v.y) + T.position.y;

	return p2Vec2(x, y);
}

// Perform the dot product on two vectors.
inline float p2Dot(const p2Vec2& a, const p2Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}