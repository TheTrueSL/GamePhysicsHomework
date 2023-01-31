#pragma once

#include "Collider.h"

#include <memory>

class DrawingUtilitiesClass;

namespace GamePhysics {
	class GameObject
	{
	public:
		GameObject();
		GameObject(Transform* transform, Rigidbody* body, Collider* collider);
		~GameObject();

		void regist(Transform* transform, Rigidbody* body, Collider* collider);

		void draw(DrawingUtilitiesClass* duc);
		void update();

		std::shared_ptr<Transform> transform;
		std::shared_ptr<Rigidbody> rigidbody;
		std::shared_ptr<Collider> collider;
	};

	class CustomObject : public GameObject {
	public:
		virtual void onCollisionEnter(Collider* other)=0;
		virtual void onDraw(DrawingUtilitiesClass* duc) = 0;
		virtual void onFrameUpdate() = 0;
		virtual void onPhysicsUpdate(float dt) = 0;
	};

	
}
