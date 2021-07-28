#pragma once
#include "p2_math.h"

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
struct p2Rigidbody
{
	enum Type {
		staticBody,
		kinematicBody,
		dynamicBody,
	};

	p2Rigidbody();

	Type type;

	p2Vec2 position;
	float rotation;

	p2Vec2 linearVelocity;
	float angularVelocity;

	p2Vec2 force;
	float torque;

	float mass, invMass;

	float inertia, invInertia;

	float friction, restitution;

	p2Transform GetTransform() const;
};

