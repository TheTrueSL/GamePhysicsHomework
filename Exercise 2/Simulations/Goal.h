#pragma once

#include "GameObject.h"

class GameManager;
namespace GamePhysics {

	class Goal : public CustomObject {
	public:
		Goal(GameManager* gm);
		bool scored = false;
		void onCollisionEnter(Collider* other);
		void onDraw(DrawingUtilitiesClass* duc);
		void onFrameUpdate();
		void onPhysicsUpdate(float dt);

		GameManager* gameManager;
	};

}

