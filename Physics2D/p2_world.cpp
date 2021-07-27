#include "p2_world.h"

p2World::p2World() : shapes(), shapeCount(0), contactManager()
{
}

p2World::~p2World()
{
	for (int i = 0; i < shapeCount; i++) {
		delete shapes[i];
	}
}

p2PolygonShape* p2World::CreatePolygonShape()
{
	// shapes[shapeCount] = new p2PolygonShape();
	// return shapes[shapeCount++];
	return nullptr;
}

p2CircleShape* p2World::CreateCircleShape(float radius)
{
	shapes[shapeCount] = new p2CircleShape(radius);
	return shapes[shapeCount++];
}

void p2World::Step()
{
	std::vector<p2Contact> contacts = contactManager.GetContacts(shapes);

}
