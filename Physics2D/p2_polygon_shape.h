#pragma once
#include "p2_shape.h"

class p2PolygonShape : public p2Shape
{
public:
	p2PolygonShape();
	~p2PolygonShape();

	// Returns number of children shapes
	int GetChildrenCount() const override;

	/// <summary>
	/// Computs the mass data of all the children and inertia relative to the local center
	/// </summary>
	/// <param name="massData"></param>
	/// <param name="density"></param>
	void ComputeMass(p2MassData* massData, float density) const override;

	/// <summary>
	/// Returns an aabb for the child at the given index at the give world transform
	/// </summary>
	void ComputeAABB(p2AABB* aabb, p2Transform wTransform, int childIndex) const override;

	/// Cast a ray against a child shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	/// @param transform the transform to be applied to the shape.
	/// @param childIndex the child shape index
	bool RayCast(p2RayCastOutput* output, const p2RayCastInput& input, const p2Transform& transform, int childIndex) const override;

	void SetRect(float hx, float hy);
private:
	p2Vec2 centroid;
	p2Vec2 vertices[4];
	p2Vec2 normals[4];
	const int count = 4;
};

