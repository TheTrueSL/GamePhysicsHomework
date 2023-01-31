#include "Ball.h"

#include "DrawingUtilitiesClass.h"

void GamePhysics::Ball::onCollisionEnter(Collider* other)
{
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
}
