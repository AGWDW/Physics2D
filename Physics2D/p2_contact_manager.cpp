#include "p2_contact_manager.h"
#include "p2_object.h"

std::vector<p2Contact> p2ContactManager::GetContacts(std::array<p2CircleShape*, p2_Max_Objects> shapes)
{
	std::vector<p2Contact> res;
	res = Broadphase(shapes);
	Narrowphase(res);
	return res;
}

std::vector<p2Contact> p2ContactManager::Broadphase(std::array<p2CircleShape*, p2_Max_Objects> shapes)
{
	std::vector<p2Contact> res;
	for (int i = 0; i < shapes.size(); i++) {
		for (int j = i + 1; j < shapes.size(); j++) {
			p2CircleShape* a = shapes[i];
			p2CircleShape* b = shapes[j];

			// if nullptr
			if (!a || !b) {
				continue;
			}

			// if the same shape
			if (a == b) {
				continue;
			}

			p2AABB aabb1;
			p2AABB aabb2;
			p2Transform transformA = a->parent->body->GetTransform();
			p2Transform transformB = b->parent->body->GetTransform();
			a->ComputeAABB(&aabb1, transformA, 0);
			b->ComputeAABB(&aabb2, transformB, 0);

			bool overlap = aabb1.Overlaps(aabb2) || aabb2.Overlaps(aabb1);
			if (overlap) {
				p2Contact contact;
				contact.shapeA = a;
				contact.shapeB = b;
				contact.bodyA = a->parent->body;
				contact.bodyB = b->parent->body;
				contact.transA = transformA;
				contact.transB = transformB;
				res.push_back(contact);
			}
		}
	}
	return res;
}

void p2ContactManager::Narrowphase(std::vector<p2Contact>& contacts)
{
	for (auto itt = contacts.begin(); itt != contacts.end();) {
		p2Contact& contact = *itt;
		contact.type = p2Contact::circle_circle;

		p2CircleShape* a = contact.shapeA;
		p2CircleShape* b = contact.shapeB;

		p2Transform fxA = contact.transA;
		p2Transform fxB = contact.transB;

		p2Vec2 d = fxB.position - fxA.position;
		float d_sqrd = p2Dot(d, d);

		if (d_sqrd < p2_Epsilon * p2_Epsilon) {
			itt = contacts.erase(itt);
			continue;
		}

		p2Vec2& normal = contact.normal;
		normal = fxB.position - fxA.position;
		normal.Normalize();

		p2Vec2 cA = fxA.position + normal * a->radius;
		p2Vec2 cB = fxB.position - normal * b->radius;

		contact.numPoints = 1;
		contact.points[0].point = 0.5f * (cA + cB);
		contact.points[0].separation = p2Dot(cB - cA, normal);
		itt++;

	}
}
