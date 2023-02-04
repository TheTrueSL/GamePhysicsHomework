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
		//transform->position = Vec3(0.5, -0.4, 0);
		transform->position = Vec3(0.5, 0, 0);
		transform->rotation = Quat(0, 0, 0, 1);

		Rigidbody* rigidbody = new Rigidbody(transform);
		rigidbody->friction = 0.5;
		rigidbody->angularVelocity = Vec3(1, 0, 0);
		rigidbody->angularMomentum = Vec3(1,0, 0);
		rigidbody->setSphereInertia(0.25, 0.1);

		Collider* collider = new Collider(transform, rigidbody);
		collider->drawAxis = true;
		collider->ballFlag = true;
		collider->setSphere(0.2);

		ball = new Ball();
		ball->regist(transform, rigidbody, collider);

		// binded objects will be automatically deleted by simulator when reset() is called;
		simulator->bindCustomObject(ball);
	}
	//Goal Detection Box Setup
	{
		
		Transform* transform = new Transform();
		transform->position = Vec3(0, 0.6, 6.5);
		transform->rotation = Quat(0, 0, 1, 1);
		transform->rotation /= transform->rotation.norm();
		Rigidbody* rigidbody = new Rigidbody(transform);
		
		Collider* collider = new Collider(transform, rigidbody);	
		collider->isTrigger = true;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(2.5, 7.5, 0.2));
		goal = new Goal();
		goal->regist(transform, rigidbody, collider);
		simulator->bindCustomObject(goal);
	}
	//Create Goal
	{
		Transform* transform = new Transform();
		transform->position = Vec3(0, 1.75, 6.5);
		transform->rotation = Quat(0, 0, 1, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 7.5, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Top Back Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(0, 1.75, 8);
		transform->rotation = Quat(0, 0, 1, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 7.5, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Right Front Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(-3.75, 0.6, 6.5);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Right Back Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(-3.75, 0.6, 8);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Left Front Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(3.75, 0.6, 6.5);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Left Back Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(3.75, 0.6, 8);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	{
		//Backside of the grid
		for(int i = 0; i < 15;  i++) {
			for (int j = 0; j < 4; j++) {
				Transform* t1 = new Transform();
				t1->position = Vec3(-3.5 + (0.5 * i), 1.3-(0.5*j), 8);
				Rigidbody* rb1 = new Rigidbody(t1);
				rb1->isFixed = true;
				Collider* col1 = new Collider(t1, rb1);
				col1->layer = 0b1000;
				col1->filter = 0b0010;
				col1->setBox(Vec3(0.4, 0.4, 0.01));
				GameObject* newPoint = new GameObject(t1, rb1, col1);
				masspoints.push_back(newPoint);
				simulator->bindGameObject(newPoint);
			}
		}
	}
}



void GameManager::onFrameUpdate()
{
}
