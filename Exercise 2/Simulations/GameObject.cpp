#include "GameObject.h"

GamePhysics::GameObject::GameObject()
{
}

GamePhysics::GameObject::GameObject(Transform* transform, Rigidbody* body, Collider* collider)
{
	regist(transform, body, collider);
}

GamePhysics::GameObject::~GameObject()
{
	transform.reset();
	rigidbody.reset();
	collider.reset();
}

void GamePhysics::GameObject::regist(Transform* transform, Rigidbody* body, Collider* collider)
{
	this->transform = std::shared_ptr<Transform>(transform);
	this->rigidbody = std::shared_ptr<Rigidbody>(body);
	this->collider = std::shared_ptr<Collider>(collider);
}

void GamePhysics::GameObject::draw(DrawingUtilitiesClass* duc)
{
	if (collider) {
		collider->draw(duc);
	}
}

void GamePhysics::GameObject::update()
{
	if (transform) {
		transform->update();
	}
	if (collider) {
		collider->update();
	}
	if (rigidbody) {
		rigidbody->update();
	}
}
