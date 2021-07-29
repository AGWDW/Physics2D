#pragma once
#include "p2_shape.h"
#include "p2_polygon_shape.h"
#include "p2_circle_shape.h"
#include "p2_settings.h"
#include "p2_contact_manager.h"
#include "p2_object.h"
#include <array>

class p2World
{
public:
	p2World(float gravity);
	~p2World();
	p2Object* CreateObject();
	p2Rigidbody* CreateBody();
	p2CircleShape* CreateCircleShape(float radius);
	void InitalizePositionalVelocityConstraints(std::vector<p2Contact>& contacts, std::vector<b2ContactVelocityConstraint>& constraintsVel, std::vector<b2ContactPositionConstraint>& constraintsPos) const;
	void SolveVelocityConstraints(std::vector<b2ContactVelocityConstraint>& constraints) const;
	bool SolvePositionalConstraints(std::vector<b2ContactPositionConstraint>& constraints) const;
	void Step(float step);
private:
	std::array<p2Object*, p2_Max_Objects> objects;
	std::array<p2Rigidbody*, p2_Max_Objects> bodys;
	std::array<p2CircleShape*, p2_Max_Shapes> shapes;
	int objectCount, shapeCount, bodyCount;
	p2ContactManager contactManager;
	float gravity;
};

