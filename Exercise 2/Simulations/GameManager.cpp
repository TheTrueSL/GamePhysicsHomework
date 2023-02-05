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
		collider->layer = 0b100;
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
		float  length = 0.01f;
		Vec3 offset1 = Vec3(0., 0.2, 0);
		Vec3 offset2 = Vec3(0, -0.2, 0);

		Rigidbody* lastLine1 = NULL;
		Rigidbody* lastLine2 = NULL;
		Rigidbody* lastLine3 = NULL;
		Rigidbody* lastLine4 = NULL;
		{
			Transform* t = new Transform();
			t->position = Vec3(-3.75, 1.3, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110; 
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			lastLine1 = rb;
		}
		{
			Transform* t = new Transform();
			t->position = Vec3(-3.75, 0.8, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110; 
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			lastLine2 = rb;
		}
		{
			Transform* t = new Transform();
			t->position = Vec3(-3.75, 0.3, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110; 
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			lastLine3 = rb;
		}
		{
			Transform* t = new Transform();
			t->position = Vec3(-3.75, -0.2, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110; 
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			lastLine4 = rb;
		}

		for (int i = 0; i < 15; i++) {
			Transform* t = new Transform();
			t->position = Vec3(-3.5 + (0.5 * i), 1.75, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110;
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			if (oldPoint != NULL) {
				simulator->bindSpring(
					new Spring(oldPoint, rb, stiffness, dampening, length, offset1, offset2));
			}
			oldPoint = rb;

			for (int j = 0; j < 4; j++) {
				Transform* t = new Transform();
				t->position = Vec3(-3.5 + (0.5 * i), 1.3 - (0.5 * j), 8);
				Rigidbody* rb = new Rigidbody(t);
				//Change here to false to activate physics
				rb->isFixed = false;
				rb->useGravity = false;
				rb->mass = 0.2f;
				Collider* col = new Collider(t, rb);
				col->layer = 0b1000;
				col->filter = 0b0110;
				col->setBox(Vec3(0.4, 0.4, 0.01));
				GameObject* newPoint = new GameObject(t, rb, col);
				simulator->bindGameObject(newPoint);
				if (oldPoint != NULL) {
					simulator->bindSpring(
						new Spring(oldPoint, rb, stiffness, dampening, length, offset1, offset2));
				}
				oldPoint = rb;
				switch (j)
				{
				case 0:
					if (lastLine1 != NULL) {
						simulator->bindSpring(
							new Spring(lastLine1, rb, stiffness, dampening, length, offset1, offset2));
					}
					lastLine1 = rb;
					break;
				case 1:
					if (lastLine2 != NULL) {
						simulator->bindSpring(
							new Spring(lastLine2, rb, stiffness, dampening, length, offset1, offset2));
					}
					lastLine2 = rb;
					break;
				case 2:
					if (lastLine3 != NULL) {
						simulator->bindSpring(
							new Spring(lastLine3, rb, stiffness, dampening, length, offset1, offset2));
					}
					lastLine3 = rb;
					break;
				case 3:
					if (lastLine4 != NULL) {
						simulator->bindSpring(
							new Spring(lastLine4, rb, stiffness, dampening, length, offset1, offset2));
					}
					lastLine4 = rb;
					break;
				default:
					break;
				}
			}
			Transform* t2 = new Transform();
			t2->position = Vec3(-3.5 + (0.5 * i), -0.5, 8);
			Rigidbody* rb2 = new Rigidbody(t2);
			rb2->isFixed = true;
			Collider* col2 = new Collider(t2, rb2);
			col2->layer = 0b1000;
			col2->filter = 0b0110;
			col2->setSphere(0.01);
			GameObject* newPoint2 = new GameObject(t2, rb2, col2);
			simulator->bindGameObject(newPoint2);
			if (oldPoint != NULL) {
				simulator->bindSpring(
					new Spring(oldPoint, rb2, stiffness, dampening, length, offset1, offset2));
			}
			oldPoint = NULL;
		}
		{
			Transform* t = new Transform();
			t->position = Vec3(3.75, 1.3, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110;
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			simulator->bindSpring(
				new Spring(lastLine1, rb, stiffness, dampening, length, offset1, offset2));
		}
		{
			Transform* t = new Transform();
			t->position = Vec3(3.75, 0.8, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110;
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			simulator->bindSpring(
				new Spring(lastLine2, rb, stiffness, dampening, length, offset1, offset2));
		}
		{
			Transform* t = new Transform();
			t->position = Vec3(3.75, 0.3, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110;
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			simulator->bindSpring(
				new Spring(lastLine3, rb, stiffness, dampening, length, offset1, offset2));
		}
		{
			Transform* t = new Transform();
			t->position = Vec3(3.75, -0.2, 8);
			Rigidbody* rb = new Rigidbody(t);
			rb->isFixed = true;
			Collider* col = new Collider(t, rb);
			col->layer = 0b1000;
			col->filter = 0b0110;
			col->setSphere(0.1);
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
			simulator->bindSpring(
				new Spring(lastLine4, rb, stiffness, dampening, length, offset1, offset2));
		}

	}
}



void GameManager::onFrameUpdate()
{
}
