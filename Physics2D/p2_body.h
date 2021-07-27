#pragma once
#include "p2_math.h"

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum p2_bodyType {
	staticBody,
	kinematicBody,
	dynamicBody,
};

class p2Rigidbody
{
private:
	p2_bodyType bodyType;

	p2Vec2 positions;
	float rotations;

	p2Vec2 linearVelocity;
	float angularVelocity;

	p2Vec2 force;
	float torque;

	float mass, invMass;

	float inertia, invInertia;
};

