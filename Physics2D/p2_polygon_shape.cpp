#include "p2_polygon_shape.h"

p2PolygonShape::p2PolygonShape() : centroid(0, 0)
{
}

p2PolygonShape::~p2PolygonShape()
{
	delete[] vertices;
	delete[] normals;
}

int p2PolygonShape::GetChildrenCount() const
{
    return 1;
}

void p2PolygonShape::ComputeMass(p2MassData* massData, float density) const
{
}

void p2PolygonShape::ComputeAABB(p2AABB* aabb, p2Transform wTransform, int childIndex) const
{
	p2Vec2 lower = b2Mul(wTransform, vertices[0]);
	p2Vec2 upper = lower;

	for (int i = 1; i < count; ++i)
	{
		p2Vec2 v = b2Mul(wTransform, vertices[i]);
		lower = p2Min(lower, v);
		upper = p2Max(upper, v);
	}

	p2Vec2 r(0, 0);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

bool p2PolygonShape::RayCast(p2RayCastOutput* output, const p2RayCastInput& input, const p2Transform& transform, int childIndex) const
{
	return false;
}

void p2PolygonShape::SetRect(float hx, float hy)
{
	vertices[0].Set(-hx, -hy);
	vertices[1].Set(hx, -hy);
	vertices[2].Set(hx, hy);
	vertices[3].Set(-hx, hy);

	normals[0].Set(0.0f, -1.0f);
	normals[1].Set(1.0f, 0.0f);
	normals[2].Set(0.0f, 1.0f);
	normals[3].Set(-1.0f, 0.0f);

	centroid.SetZero();
}
