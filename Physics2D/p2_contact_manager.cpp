#include "p2_contact_manager.h"

std::vector<p2Contact> p2ContactManager::GetContacts(std::array<p2CircleShape*, p2_Max_Polygons> shapes)
{
	std::vector<p2Contact> res;
	res = Broadphase(shapes);
	Narrowphase(res);
	return res;
}

std::vector<p2Contact> p2ContactManager::Broadphase(std::array<p2CircleShape*, p2_Max_Polygons> shapes)
{
	std::vector<p2Contact> res;
	for (int i = 0; i < shapes.size(); i++) {
		for (int j = i; j < shapes.size(); j++) {
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
			p2Transform transform;
			a->ComputeAABB(&aabb1, transform, 0);
			b->ComputeAABB(&aabb2, transform, 0);

			bool overlap = aabb1.Overlaps(aabb2) || aabb2.Overlaps(aabb1);
			if (overlap) {
				p2Contact contact;
				contact.shapeA = a;
				contact.shapeB = b;
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

		p2Transform* fxA = contact.transA;
		p2Transform* fxB = contact.transB;

		float rA = a->radius;
		float rB = b->radius;

		if (rA * rB < p2_Epsilon * p2_Epsilon) {
			itt = contacts.erase(itt);
			continue;
		}

		p2Vec2& normal = contact.normal;
		normal = fxB->position - fxA->position;
		normal.Normalize();

		p2Vec2 cA = fxA->position + normal * rA;
		p2Vec2 cB = fxB->position - normal * rB;

		contact.points[0].point = 0.5f * (cA + cB);
		contact.points[0].separation = p2Dot(cB - cA, normal);
		itt++;

	}
}
