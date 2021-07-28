#include "p2_world.h"
#include "p2_object.h"

void main() {
	p2World world(9.81f);

	p2Object* objA = world.CreateObject();
	objA->shape = world.CreateCircleShape(10);
	objA->shape->parent = objA;
	objA->body = world.CreateBody();

	objA->body->type = p2Rigidbody::dynamicBody;

	objA->body->mass = 10.0f;
	objA->body->invMass = 1.0f / objA->body->mass;
	   
	objA->body->inertia = 10.0f;
	objA->body->invInertia = 1.0f / objA->body->inertia;
	objA->body->position.Set(-5, 0);


	p2Object* objB = world.CreateObject();
	objB->shape = world.CreateCircleShape(10);
	objB->shape->parent = objB;
	objB->body = world.CreateBody();

	objB->body->type = p2Rigidbody::dynamicBody;

	objB->body->mass = 10.0f;
	objB->body->invMass = 1.0f / objB->body->mass;

	objB->body->inertia = 10.0f;
	objB->body->invInertia = 1.0f / objB->body->inertia;
	objB->body->position.Set(5, 0);


	const float timeStep = 1.0f / 60.f;

	world.Step(timeStep);
}
