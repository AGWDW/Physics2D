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

void p2World::InitalizePositionalVelocityConstraints(std::vector<p2Contact>& contacts, std::vector<b2ContactVelocityConstraint>& constraintsVel, 
	std::vector<b2ContactPositionConstraint>& constraintsPos) const
{
	for (int i = 0; i < contacts.size(); i++) {
		b2ContactPositionConstraint constraintPos;
		b2ContactVelocityConstraint constraintVel;

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


		constraintPos.invMassA = mA;
		constraintPos.invMassB = mB;

		constraintPos.localCenterA.SetZero();
		constraintPos.localCenterB.SetZero();

		constraintPos.invIA = iA;
		constraintPos.invIB = iB;

		// constraintPos.localNormal = manifold->localNormal; not used for circles

		constraintPos.localPoint = constraintPos.localCenterA; // center of A
		constraintPos.pointCount = contact.numPoints;

		constraintPos.radiusA = contact.shapeA->radius;
		constraintPos.radiusB = contact.shapeB->radius;

		constraintVel.invMassA = mA;
		constraintVel.invMassB = mB;

		constraintVel.invIA = iA;
		constraintVel.invIB = iB;

		constraintVel.vA = &vA;
		constraintVel.wB = &wB;

		constraintVel.vA = &vA;
		constraintVel.wB = &wB;

		constraintVel.normal = contact.normal;
		constraintVel.friction = contact.friction;
		constraintVel.restitution = contact.restitution;

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
			constraintVel.points[j] = constraintPoint;
			constraintVel.pointCount = j + 1;

			constraintPos.localPoints[j] = point.point;
		}
		constraintsVel.push_back(constraintVel);
		constraintsPos.push_back(constraintPos);
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

bool p2World::SolvePositionalConstraints(std::vector<b2ContactPositionConstraint>& constraints) const
{
	float minSeparation = 0.0f;

	for (int i = 0; i < constraints.size(); ++i)
	{
		b2ContactPositionConstraint* pc = &constraints[i];

		p2Vec2 localCenterA = pc->localCenterA;
		float mA = pc->invMassA;
		float iA = pc->invIA;
		p2Vec2 localCenterB = pc->localCenterB;
		float mB = pc->invMassB;
		float iB = pc->invIB;
		int pointCount = pc->pointCount;

		p2Vec2 cA = m_positions[indexA].c;
		float aA = m_positions[indexA].a;

		p2Vec2 cB = m_positions[indexB].c;
		float aB = m_positions[indexB].a;

		// Solve normal constraints
		for (int j = 0; j < pointCount; ++j)
		{
			p2Transform xfA, xfB;
			xfA.rotation.Set(aA);
			xfB.rotation.Set(aB);
			xfA.position = cA - b2Mul(xfA.rotation, localCenterA);
			xfB.position = cB - b2Mul(xfB.rotation, localCenterB);

			b2PositionSolverManifold psm;
			psm.Initialize(pc, xfA, xfB, j);
			p2Vec2 normal = psm.normal;

			p2Vec2 point = psm.point;
			float separation = psm.separation;

			p2Vec2 rA = point - cA;
			p2Vec2 rB = point - cB;

			// Track max constraint error.
			minSeparation = p2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float C = p2Clamp(p2_baumgarte * (separation + p2_linearSlop), -p2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float rnA = p2Cross(rA, normal);
			float rnB = p2Cross(rB, normal);
			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float impulse = K > 0.0f ? -C / K : 0.0f;

			p2Vec2 P = impulse * normal;

			cA -= mA * P;
			aA -= iA * p2Cross(rA, P);

			cB += mB * P;
			aB += iB * p2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0f * p2_linearSlop;
}

void p2World::Step(float step)
{
	std::vector<p2Contact> contacts = contactManager.GetContacts(shapes);

	std::vector<b2ContactVelocityConstraint> constraintsVel;
	std::vector<b2ContactPositionConstraint> constraintsPos;
	InitalizePositionalVelocityConstraints(contacts, constraintsVel, constraintsPos);
}
