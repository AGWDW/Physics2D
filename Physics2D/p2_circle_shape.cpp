#include "p2_circle_shape.h"

p2CircleShape::p2CircleShape() : p2Shape()
{
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
	float h_r = radius * 0.5f;
	aabb->lowerBound.Set(-h_r, -h_r);
	aabb->upperBound.Set(h_r, h_r);
}

bool p2CircleShape::RayCast(p2RayCastOutput* output, const p2RayCastInput& input, const p2Transform& transform, int childIndex) const
{
	return false;
}
