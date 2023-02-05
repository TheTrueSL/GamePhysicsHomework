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
		rigidbody->velocity = Vec3(0.1, 0.1, 4);
		rigidbody->angularVelocity = Vec3(1, 0, 0);
		rigidbody->angularMomentum = Vec3(0.1, 0.1, 0.0);
		rigidbody->setSphereInertia(0.25, 0.1);

		Collider* collider = new Collider(transform, rigidbody);
		collider->drawAxis = true;
		collider->layer = 0b100;
		collider->ballFlag = true;
		collider->setSphere(0.2);

		ball = new Ball();
		ball->regist(transform, rigidbody, collider);
		// binded objects will be automatically deleted by simulator when reset() is called;
		simulator->bindCustomObject(ball);
		player->attachBall(ball);
	}
	//Goal Detection Box Setup
	{
		
		Transform* transform = new Transform();
		transform->position = Vec3(0, 0.6, 6.5);
		transform->rotation = Quat(0, 0, 1, 1);
		transform->rotation /= transform->rotation.norm();
		Rigidbody* rigidbody = new Rigidbody(transform);
		
		Collider* collider = new Collider(transform, rigidbody);	
		collider->layer = 0b100;
		collider->filter = 0b0100;
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
		collider->layer = 0b100;
		collider->filter = 0b100;
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
		collider->layer = 0b100;
		collider->filter = 0b100;
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
		collider->layer = 0b100;
		collider->filter = 0b100;
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
		collider->layer = 0b100;
		collider->filter = 0b100;
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
		collider->layer = 0b100;
		collider->filter = 0b100;
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
		collider->layer = 0b100;
		collider->filter = 0b100;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Top Surface
	{
		Transform* t = new Transform();
		t->position = Vec3(0, 1.75, 7.25);
		t->rotation = Quat(1, 0, 0, 1);
		t->rotation /= t->rotation.norm();
		Rigidbody* rb = new Rigidbody(t);
		rb->isFixed = true;
		Collider* col = new Collider(t, rb);
		col->layer = 0b100;
		col->filter = 0b100;
		col->setBox(Vec3(7.5, 1.25, 0.01));
		GameObject* newPoint = new GameObject(t, rb, col);
		simulator->bindGameObject(newPoint);
	}
	//Left Surface
	{
		Transform* t = new Transform();
		t->position = Vec3(-3.75, 0.6, 7.25);
		t->rotation = Quat(0, 1, 0, 1);
		t->rotation /= t->rotation.norm();
		Rigidbody* rb = new Rigidbody(t);
		rb->isFixed = true;
		Collider* col = new Collider(t, rb);
		col->layer = 0b100;
		col->filter = 0b100;
		col->setBox(Vec3(1.7, 2.25, 0.01));
		GameObject* newPoint = new GameObject(t, rb, col);
		simulator->bindGameObject(newPoint);
	}
	//Right Surface
	{
		Transform* t = new Transform();
		t->position = Vec3(3.75, 0.6, 7.25);
		t->rotation = Quat(0, 1, 0, 1);
		t->rotation /= t->rotation.norm();
		Rigidbody* rb = new Rigidbody(t);
		rb->isFixed = true;
		Collider* col = new Collider(t, rb);
		col->layer = 0b100;
		col->filter = 0b100;
		col->setBox(Vec3(1.7, 2.25, 0.01));
		GameObject* newPoint = new GameObject(t, rb, col);
		simulator->bindGameObject(newPoint);
	}

	{
		//Backside of the grid
		Rigidbody* oldPoint = NULL;
		float stiffness = 1.0f;
		float dampening = 1.0f;
		float  length = 0.2f;
		float mass = 0.2f;

		for (int i = 0; i < 13; i++) {
			for (int j = 0; j < 38; j++) {
				Transform* t = new Transform();
				t->position = Vec3(-3.75 + (0.2 * j), 1.75 - (0.2 * i), 8);
				Rigidbody* rb = new Rigidbody(t);
				//Change here to false to activate physics
				if (i == 0 || i == 12 || j == 0 || j == 37) {
					rb->isFixed = true;
				}
				else {
					rb->isFixed = false;
				}
				rb->useGravity = false;
				rb->mass = mass;
				Collider* col = new Collider(t, rb);
				col->layer = 0b1000;
				col->filter = 0b0110;
				col->setSphere(0.01);
				GameObject* newPoint = new GameObject(t, rb, col);
				netGrid.push_back(rb);
				simulator->bindGameObject(newPoint);
				if (!j == 0) {
					simulator->bindSpring(new Spring(oldPoint, rb, stiffness, dampening, length, Vec3(0, 0, 0), Vec3(0, 0, 0)));
				}
				oldPoint = rb;
			}
		}

		for (int z=38; z<netGrid.size()-1; z++)
		{
			Rigidbody* rb1 = netGrid[z];
			Rigidbody* rb2 = netGrid[z-38];


			simulator->bindSpring(new Spring(rb1, rb2, stiffness, dampening, length, Vec3(0, 0, 0), Vec3(0, 0, 0)));
		}

	}
}



void GameManager::onFrameUpdate()
{
}
