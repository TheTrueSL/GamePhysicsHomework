#pragma once

#include "GameObject.h"

class GameManager;

namespace GamePhysics {

	class Net : public CustomObject {
	public:
		Net(GameManager*);
		void onCollisionEnter(Collider* other);
		void onDraw(DrawingUtilitiesClass* duc);
		void onFrameUpdate();
		void onPhysicsUpdate(float dt);

		std::vector<Transform*> ts;
	};

}