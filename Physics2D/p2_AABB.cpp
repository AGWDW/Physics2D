#include "p2_AABB.h"

bool p2AABB::IsValid() const
{
	return upperBound.x > lowerBound.x && upperBound.y > lowerBound.y;
}

bool p2AABB::RayCast(p2RayCastOutput* output, const p2RayCastInput& input) const
{
	return false;
}

bool p2AABB::Overlaps(p2AABB& b) const
{
	const p2AABB& a = *this;
	p2Vec2 d1, d2;
	d1 = b.lowerBound - a.upperBound;
	d2 = a.lowerBound - b.upperBound;

	if (d1.x > 0.0f || d1.y > 0.0f) {
		return false;
	}

	if (d2.x > 0.0f || d2.y > 0.0f) {
		return false;
	}

	return true;
}
