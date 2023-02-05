#include "Ball.h"

#include "DrawingUtilitiesClass.h"

void GamePhysics::Ball::onCollisionEnter(Collider* other)
{
	if (other != nullptr) {
		rigidbody->angularMomentum *= 0.99;
	}
}

void GamePhysics::Ball::onDraw(DrawingUtilitiesClass* duc)
{
	Mat4 scale;
	scale.initScaling(collider->size.x, collider->size.x, collider->size.x);
	duc->drawBall(scale * transform->transformation);
}

void GamePhysics::Ball::onFrameUpdate()
{
}

void GamePhysics::Ball::onPhysicsUpdate(float dt)
{
	if (transform->position.y > -0.5 + collider->size.x) {
		// on air
		rigidbody->force += cross(rigidbody->velocity, rigidbody->angularVelocity) * dt * 0.7;
	}
	else {
		rigidbody->force += cross(rigidbody->velocity, rigidbody->angularVelocity) * dt * 0.4;
	}
}
