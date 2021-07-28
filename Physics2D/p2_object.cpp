#include "p2_object.h"

p2Object::p2Object() : shape(nullptr), body(nullptr)
{

}

p2Object::~p2Object()
{
	delete shape;
	delete body;
}
