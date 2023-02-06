#pragma once

#include "GameObject.h"

namespace GamePhysics {

	class Ball : public CustomObject {
	public:
		void onCollisionEnter(Collider* other);
		void onDraw(DrawingUtilitiesClass* duc);
		void onFrameUpdate();
		void onPhysicsUpdate(float dt);
	};

}
