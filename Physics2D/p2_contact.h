#pragma once
#include <array>
#include "p2_circle_shape.h"
#include "p2_settings.h"

class p2Rigidbody;
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
	p2Rigidbody* bodyA, *bodyB;
	p2CircleShape* shapeA, *shapeB;
	p2Transform transA, transB;
	p2Vec2 normal;
	Type type;
	int numPoints;
	float friction;
	float restitution;
	float threshold;

	std::array<p2ContactPoint, p2_Max_Points> points;

	p2Contact() : bodyA(nullptr), bodyB(nullptr), shapeA(nullptr), shapeB(nullptr), transA(), transB(), 
		normal(), type(circle_circle), numPoints(0), friction(0), restitution(0), threshold(0.1f), points() {

	}

	~p2Contact() {
	}
};

