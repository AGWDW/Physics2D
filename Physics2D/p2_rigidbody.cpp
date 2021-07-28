#include "p2_rigidbody.h"

p2Rigidbody::p2Rigidbody() : type(staticBody), position(), rotation(0), linearVelocity(), angularVelocity(0), 
	force(), torque(0), mass(0), invMass(0), inertia(0), invInertia(0), friction(0), restitution(0)
{

}

p2Transform p2Rigidbody::GetTransform() const
{
	p2Rot rot(rotation);
	p2Transform res(position, rot);
	return res;
}
