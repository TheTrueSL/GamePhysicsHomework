#include "Goal.h"

#include "DrawingUtilitiesClass.h"

void GamePhysics::Goal::onCollisionEnter(Collider* other)
{

	if (other !=NULL && other->ballFlag && !scored) {
		std::cout << "GOAL!";
		scored = true;
	}
}

void GamePhysics::Goal::onDraw(DrawingUtilitiesClass* duc)
{
}

void GamePhysics::Goal::onFrameUpdate()
{
}

void GamePhysics::Goal::onPhysicsUpdate(float dt)
{
}