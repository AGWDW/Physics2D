#pragma once
#include "p2_math.h"
#include "p2_raycast.h"

struct p2AABB
{
	p2AABB() : lowerBound(), upperBound() { };
	~p2AABB() { };

	/// Verify that the bounds are sorted.
	bool IsValid() const;

	/// Get the center of the AABB.
	p2Vec2 GetCenter() const {
		return 0.5f * (lowerBound + upperBound);
	};

	/// Get the extents of the AABB (half-widths).
	p2Vec2 GetExtent() const {
		return 0.5f * (upperBound - lowerBound);
	}

	/// Get the perimeter length
	float GetPerimeter() const {
		p2Vec2 d = upperBound - lowerBound;
		return 2.0f * (d.x + d.y);
	}

	/// Combine an AABB into this one.
	void Combine(const p2AABB& aabb)
	{
		lowerBound = p2Min(lowerBound, aabb.lowerBound);
		upperBound = p2Max(upperBound, aabb.upperBound);
	}

	/// Combine two AABBs into this one.
	void Combine(const p2AABB& aabb1, const p2AABB& aabb2)
	{
		lowerBound = p2Min(aabb1.lowerBound, aabb2.lowerBound);
		upperBound = p2Max(aabb1.upperBound, aabb2.upperBound);
	}

	/// Does this aabb contain the provided AABB.
	bool Contains(const p2AABB& aabb) const
	{
		bool result = true;
		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;
		return result;
	}

	bool RayCast(p2RayCastOutput* output, const p2RayCastInput& input) const;

	bool Overlaps(p2AABB& b) const;

	p2Vec2 upperBound;
	p2Vec2 lowerBound;
};
