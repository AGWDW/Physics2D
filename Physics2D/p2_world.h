#pragma once
#include "p2_shape.h"
#include "p2_polygon_shape.h"
#include "p2_circle_shape.h"
#include "p2_settings.h"
#include "p2_contact_manager.h"
#include <array>

class p2World
{
public:
	p2World();
	~p2World();
	p2PolygonShape* CreatePolygonShape();
	p2CircleShape* CreateCircleShape(float radius);
	void Step();
private:
	std::array<p2CircleShape* , p2_Max_Polygons> shapes;
	int shapeCount;
	p2ContactManager contactManager;
};

