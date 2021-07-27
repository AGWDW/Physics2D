#include "math.h"
#include "p2_world.h"

void main() {
	p2World world;
	p2PolygonShape* shapeA = world.CreatePolygonShape();
	shapeA->SetRect(5, 5); // 10 x 10 box
}
