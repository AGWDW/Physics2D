#include "p2_world.h"
#include "p2_contact_constraint.h"

p2World::p2World(float gravity) : objects(), objectCount(0), contactManager(), gravity(gravity), bodys(), shapes(), shapeCount(0), bodyCount(0)
{
}

p2World::~p2World()
{
	for (int i = 0; i < objectCount; i++) {
		delete objects[i];
		objects[i] = nullptr;
	}
	for (int i = 0; i < shapeCount; i++) {
		delete shapes[i];
		shapes[i] = nullptr;
	}
	for (int i = 0; i < bodyCount; i++) {
		delete bodys[i];
		bodys[i] = nullptr;
	}
}

p2Object* p2World::CreateObject()
{
	objects[objectCount] = new p2Object();
	return objects[objectCount++];
}

p2Rigidbody* p2World::CreateBody()
{
	bodys[bodyCount] = new p2Rigidbody();
	return bodys[bodyCount++];
}

p2CircleShape* p2World::CreateCircleShape(float radius)
{
	shapes[shapeCount] = new p2CircleShape(radius);
	return shapes[shapeCount++];
}

void p2World::InitalizeVelocityConstraints(std::vector<p2Contact>& contacts, std::vector<b2ContactVelocityConstraint>& constraints) const
{
	/*
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	float threshold;
	float tangentSpeed;
	int pointCount;
	int contactIndex;
	*/
	for (int i = 0; i < contacts.size(); i++) {
		b2ContactVelocityConstraint constraint;
		p2Contact& contact = contacts[i];

		p2Vec2 cA = contact.transA.position;
		p2Vec2 cB = contact.transB.position;

		float mA = contact.bodyA->invMass;
		float mB = contact.bodyB->invMass;

		float iA = contact.bodyA->invInertia;
		float iB = contact.bodyB->invInertia;

		p2Vec2& vA = contact.bodyA->linearVelocity;
		p2Vec2& vB = contact.bodyB->linearVelocity;

		float& wA = contact.bodyA->angularVelocity;
		float& wB = contact.bodyB->angularVelocity;

		constraint.invMassA = mA;
		constraint.invMassB = mB;

		constraint.invIA = iA;
		constraint.invIB = iB;

		constraint.vA = &vA;
		constraint.wB = &wB;

		constraint.vA = &vA;
		constraint.wB = &wB;

		constraint.normal = contact.normal;
		constraint.friction = contact.friction;
		constraint.restitution = contact.restitution;

		for (int j = 0; j < contact.numPoints; j++) {
			b2VelocityConstraintPoint constraintPoint;
			p2ContactPoint& point = contact.points[j];
			constraintPoint.rA = point.point - cA;
			constraintPoint.rB = point.point - cB;

			float rnA = p2Cross(constraintPoint.rA, contact.normal);
			float rnB = p2Cross(constraintPoint.rB, contact.normal);

			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			constraintPoint.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			p2Vec2 tangent = p2Cross(contact.normal, 1.0f);

			float rtA = p2Cross(constraintPoint.rA, tangent);
			float rtB = p2Cross(constraintPoint.rB, tangent);

			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			constraintPoint.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			// Setup a velocity bias for restitution.
			constraintPoint.velocityBias = 0.0f;
			float vRel = p2Dot(contact.normal, vB + p2Cross(wB, constraintPoint.rB) - vA - p2Cross(wA, constraintPoint.rA));
			if (vRel < -contact.threshold)
			{
				constraintPoint.velocityBias = -contact.restitution * vRel;
			}
			constraint.points[j] = constraintPoint;
			constraint.pointCount = j + 1;
		}
		constraints.push_back(constraint);
	}
}

void p2World::SolveVelocityConstraints(std::vector<b2ContactVelocityConstraint>& constraints) const
{
	for (int i = 0; i < constraints.size(); i++) {
		b2ContactVelocityConstraint constraint = constraints[i];

		float mA = constraint.invMassA;
		float mB = constraint.invMassB;

		float iA = constraint.invIA;
		float iB = constraint.invIB;

		for (int j = 0; j < constraint.pointCount; ++j)
		{
			b2VelocityConstraintPoint& vcp = constraint.points[i];

			// Relative velocity at contact
			p2Vec2 dv = *constraint.vB + p2Cross(*constraint.wB, vcp.rB) - *constraint.vA - p2Cross(*constraint.wA, vcp.rA);

			// Compute normal impulse
			float vn = p2Dot(dv, constraint.normal);
			float lambda = -vcp.normalMass * (vn - vcp.velocityBias);

			// b2Clamp the accumulated impulse
			float newImpulse = p2Max(vcp.normalImpulse + lambda, 0.0f);
			lambda = newImpulse - vcp.normalImpulse;
			vcp.normalImpulse = newImpulse;

			// Apply contact impulse
			p2Vec2 P = lambda * constraint.normal;
			(*constraint.vA) -= mA * P;
			(*constraint.wA) -= iA * p2Cross(vcp.rA, P);

			(*constraint.vB) += mB * P;
			(*constraint.wB) += iB * p2Cross(vcp.rB, P);
		}
	}
}

void p2World::Step(float step)
{
	std::vector<p2Contact> contacts = contactManager.GetContacts(shapes);

	std::vector<b2ContactVelocityConstraint> constraints;
	InitalizeVelocityConstraints(contacts, constraints);

}
