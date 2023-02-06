#pragma once

#include "GameObject.h"

#include "Spring.h"

namespace GamePhysics
{
	class Ball;
	class Character
	{
	public:
		Character();
		~Character();
		void onUpdate(const float dt);
		void draw(DrawingUtilitiesClass* duc);

		void init();
		void updateTransformations();

		void onKeyPressed(unsigned int);
		void onKeyReleased(unsigned int);
		void onMouseMove(int,int, Vec3, Vec3);
		void onMousePressed(int, int, Vec3, Vec3);
		void onMouseReleased(int, int);
		void attachBall(Ball*);
		
		Ball* ball;
		void buildModel();
		
		bool planeIntersection(const Vec3& o, const Vec3& n, const Vec3& ro, const Vec3& rd, Vec3& p);
		time_t ticker;
		GameObject kicker;
		GameObject keeper;
		GameObject ancher[4];
		Spring* springs[4];
		bool pressed[5];
		float dragZoffset;
		Vec3 lastPos;
		Vec3 lastdPos;
		bool chance;
		bool enableKeeper;
	};

}