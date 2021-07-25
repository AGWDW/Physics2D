#pragma once
#include <math.h>

inline bool b2IsValid(float x)
{
	return isfinite(x);
}

struct p2Vec2 {
	/// Default constructor does nothing (for performance).
	p2Vec2() { }

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
		if (length < 0.001f)
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
	p2Vec3() { }

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