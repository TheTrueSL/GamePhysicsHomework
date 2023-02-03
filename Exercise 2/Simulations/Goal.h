#pragma once

#include "GameObject.h"

namespace GamePhysics {

	class Goal : public CustomObject {
		bool scored = false;
		void onCollisionEnter(Collider* other);
		void onDraw(DrawingUtilitiesClass* duc);
		void onFrameUpdate();
		void onPhysicsUpdate(float dt);
	};

}

