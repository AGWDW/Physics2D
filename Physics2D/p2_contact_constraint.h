#pragma once
#include <array>
#include "p2_math.h"

struct b2ContactPositionConstraint
{
	std::array<p2Vec2, p2_Max_Points> localPoints;
	p2Vec2 localNormal;
	p2Vec2 localPoint;
	int indexA;
	int indexB;
	float invMassA, invMassB;
	p2Vec2 localCenterA, localCenterB;
	float invIA, invIB;
	float radiusA, radiusB;
	int pointCount;
};

struct b2ContactVelocityConstraint
{
	std::array<b2VelocityConstraintPoint, p2_Max_Points> points;
	p2Vec2 normal;
	//p2Mat22 normalMass;
	//p2Mat22 K;
	int indexA;
	int indexB;
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	float threshold;
	float tangentSpeed;
	int pointCount;
	int contactIndex;

	p2Vec2* vA;
	p2Vec2* vB;

	float* wA;
	float* wB;
};

struct b2VelocityConstraintPoint
{
	p2Vec2 rA;
	p2Vec2 rB;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float velocityBias;
};