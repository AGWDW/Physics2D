#include "p2_circle_shape.h"

p2CircleShape::p2CircleShape() : p2Shape()
{
	type = circle;
}

p2CircleShape::p2CircleShape(float r) : p2CircleShape()
{
	radius = r;
}

int p2CircleShape::GetChildrenCount() const
{
	return 1;
}

void p2CircleShape::ComputeMass(p2MassData* massData, float density) const
{
}

void p2CircleShape::ComputeAABB(p2AABB* aabb, p2Transform wTransform, int childIndex) const
{
	aabb->lowerBound.Set(-radius, -radius);
	aabb->upperBound.Set(radius, radius);

	aabb->lowerBound += wTransform.position;
	aabb->upperBound += wTransform.position;
}

bool p2CircleShape::RayCast(p2RayCastOutput* output, const p2RayCastInput& input, const p2Transform& transform, int childIndex) const
{
	return false;
}
