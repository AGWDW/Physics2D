#pragma once
#include "p2_math.h"
#include "p2_AABB.h"

class p2Object;
struct p2MassData {
	float mass;
	p2Vec2 center;
	float inertia;
};
class p2Shape
{
public:
	enum Type {
		circle,
		poloygon,
		type_count
	};

	p2Object* parent;

	 virtual ~p2Shape() { };

	 p2Shape() : type(type_count), parent(nullptr), radius(0) { };

	 inline Type GetType() const { type; }

	 // Returns number of children shapes
	 virtual int GetChildrenCount() const = 0;

	 /// <summary>
	 /// Computs the mass data of all the children and inertia relative to the local center
	 /// </summary>
	 /// <param name="massData"></param>
	 /// <param name="density"></param>
	 virtual void ComputeMass(p2MassData* massData, float density) const = 0;

	 /// <summary>
	 /// Returns an aabb for the child at the given index at the give world transform
	 /// </summary>
	 virtual void ComputeAABB(p2AABB* aabb, p2Transform wTransform, int childIndex) const = 0;

	 /// Cast a ray against a child shape.
	 /// @param output the ray-cast results.
	 /// @param input the ray-cast input parameters.
	 /// @param transform the transform to be applied to the shape.
	 /// @param childIndex the child shape index
	 virtual bool RayCast(p2RayCastOutput* output, const p2RayCastInput& input,
		 const p2Transform& transform, int childIndex) const = 0;
	float radius;
protected:
	Type type;
};

