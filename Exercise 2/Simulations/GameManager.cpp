#include "GameManager.h"

#include "RigidBodySystemSimulator.h"

using namespace GamePhysics;

GameManager::GameManager(RigidBodySystemSimulator* simulator, Character* character)
{
	this->simulator = simulator;
	this->player = character;
}

GameManager::~GameManager()
{
}

void GameManager::onStart()
{
	player->init(Vec3(0.0, 0.3, 0));
	// ball 0
	{
		Transform* transform = new Transform();
		transform->position = Vec3(0.5, -0.4, 0);
		transform->rotation = Quat(0, 0, 0, 1);

		Rigidbody* rigidbody = new Rigidbody(transform);
		rigidbody->friction = 0.5;
		rigidbody->setSphereInertia(0.25, 0.1);

		Collider* collider = new Collider(transform, rigidbody);
		collider->drawAxis = true;
		collider->setSphere(0.1);

		ball = new Ball();
		ball->regist(transform, rigidbody, collider);

		// binded objects will be automatically deleted by simulator when reset() is called;
		simulator->bindCustomObject(ball);
	}
	//Goal Setup
	{
		
		
	}
}

void GameManager::onFrameUpdate()
{
}
