#pragma once
#include "p2_circle_shape.h"
#include "p2_settings.h"

struct p2ContactPoint {
	p2Vec2 point;
	float separation;
};

struct p2Contact
{
	enum Type {
		circle_circle,
		circle_polygon,
		polygon_polygon
	};
	p2CircleShape* shapeA, *shapeB;
	p2Transform* transA, * transB;
	p2Vec2 normal;
	Type type;

	p2ContactPoint points[p2_Max_Points];

	~p2Contact() {
		delete[] points;
	}
};

