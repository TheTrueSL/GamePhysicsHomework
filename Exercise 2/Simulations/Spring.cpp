#include "Spring.h"
#include "Rigidbody.h"

#include "DrawingUtilitiesClass.h"

using namespace GamePhysics;

std::map<int, Spring*> Spring::dict = std::map<int, Spring*>();
int Spring::id_count = 0;

Spring::Spring(Rigidbody* b0, Rigidbody* b1,
	float stiffness, float damping, float restLength,
	const Vec3& offset0, const Vec3& offset1)
{
	// append global dictionary
	this->id = Spring::id_count++;
	Spring::dict[this->id] = this;
	//
	this->b0 = b0;
	this->b1 = b1;
	
	this->offset0 = offset0;
	this->offset1 = offset1;

	this->stiffness = stiffness;
	this->damping = damping;
	this->restLength = restLength;
}

GamePhysics::Spring::~Spring()
{
	Spring::dict.erase(id);
}

void GamePhysics::Spring::applyForce()
{
	if (b0 == nullptr || b1 == nullptr)
			return;

	if (!b0->transform || !b1->transform)
		return;

	// hook's law
	Vec3 p0 = b0->transform->transformation.transformVector(offset0);
	Vec3 p1 = b1->transform->transformation.transformVector(offset1);

	Vec3 v0 = p0 - p1;
	float length = normalize(v0);
	// forces
	Vec3 f0 = -(stiffness) * (length - restLength) * v0
		- damping * dot(b0->velocity - b1->velocity, v0) * v0;

	b0->addForce(f0, p0);
	b1->addForce(-f0, p1);

	if (norm(f0) > tolerance) {
		b0 = nullptr;
		b1 = nullptr;
	}
}

void GamePhysics::Spring::drawAll(DrawingUtilitiesClass* duc)
{
	duc->beginLine();
	for (auto sit = Spring::dict.begin(); sit != Spring::dict.end(); ++sit) {
		Spring* s = sit->second;
		if (s->b0 == nullptr || s->b1 == nullptr)
			continue;
		Vec3 p0 = s->b0->transform->transformation.transformVector(s->offset0);
		Vec3 p1 = s->b1->transform->transformation.transformVector(s->offset1);
		{
			duc->drawLine(p0, Vec3(1,1,1), p1, Vec3(1, 1, 1));
		}
	}
	duc->endLine();
}
