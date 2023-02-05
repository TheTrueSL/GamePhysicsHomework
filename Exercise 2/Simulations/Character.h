#pragma once

#include "GameObject.h"

namespace GamePhysics
{
	class Character
	{
	public:
		Character();
		~Character();

		void onUpdate(const float dt);
		void draw(DrawingUtilitiesClass* duc);

		void init(Vec3 positon);
		void updateTransformations();

		void onKeyPressed(unsigned int);
		void onKeyReleased(unsigned int);
		void onMouseMove(int,int);
		void onMousePressed(int);

	private:
		void buildModel();

	};

}