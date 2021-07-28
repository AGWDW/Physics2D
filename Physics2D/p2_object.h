#pragma once
#include "p2_shape.h"
#include "p2_rigidbody.h"

struct p2Object {
	p2Shape* shape;
	p2Rigidbody* body;
	p2Object();
	~p2Object();

	bool IsValid() const {
		return body && shape;
	}
};