#include "Spring.h"
#include "Rigidbody.h"

#include "DrawingUtilitiesClass.h"

using namespace GamePhysics;

std::map<int, Spring*> Spring::dict = std::map<int, Spring*>();
int Spring::id_count = 0;

GamePhysics::Spring::Spring(Rigidbody* b0, Rigidbody* b1, float stiffness, float damping, float restLength)
{
	// append global dictionary
	this->id = Spring::id_count++;
	Spring::dict[this->id] = this;
	//
	this->b0 = b0;
	this->b1 = b1;
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
	// hook's law
	Vec3 v0 = b0->transform->position -
		b1->transform->position;
	float length = normalize(v0);
	// forces
	Vec3 f0 = -(stiffness) * (length - restLength) * v0
		- damping * dot(b0->velocity - b1->velocity, v0) * v0;
	b0->force += f0;
	b1->force += -f0;
}

void GamePhysics::Spring::drawAll(DrawingUtilitiesClass* duc)
{
	duc->beginLine();
	for (auto sit = Spring::dict.begin(); sit != Spring::dict.end(); ++sit) {
		Spring* s = sit->second;
		Transform* t0 = s->b0->transform;
		Transform* t1 = s->b1->transform;
		{
			duc->drawLine(t0->position, Vec3(1,1,1), t1->position, Vec3(1, 1, 1));
		}
	}
	duc->endLine();
}
