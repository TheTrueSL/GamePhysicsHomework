#include "GameManager.h"
#include "Goal.h"
#include "DrawingUtilitiesClass.h"

using namespace GamePhysics;

GamePhysics::Goal::Goal(GameManager* gm)
{
	this->gameManager = gm;
}

void GamePhysics::Goal::onCollisionEnter(Collider* other)
{
	if (other !=NULL && other->ballFlag && !scored) {
		std::cout << "GOAL!";
		scored = true;
		gameManager->onGameOver(true);
	}
}

void GamePhysics::Goal::onDraw(DrawingUtilitiesClass* duc)
{
	//collider->draw(duc);
}

void GamePhysics::Goal::onFrameUpdate()
{
}

void GamePhysics::Goal::onPhysicsUpdate(float dt)
{
}