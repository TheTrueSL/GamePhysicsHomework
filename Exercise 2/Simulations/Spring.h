#pragma once

#include <map>

#include "Mat3.h"

class DrawingUtilitiesClass;

namespace GamePhysics {

	class Rigidbody;

	class Spring
	{
	public:
		static std::map<int, Spring*> dict;
		static int id_count;

	public:
		Spring(Rigidbody* b0, Rigidbody* b1, 
			float stiffness, float damping, float restLength,
			const Vec3& offset0, const Vec3& offset1);
		~Spring();
		void applyForce();

		Rigidbody* b0;
		Rigidbody* b1;
		
		Vec3 offset0;
		Vec3 offset1;

		float tolerance = 1000;
		float damping;
		float stiffness;
		float restLength;

		int id;

		static void drawAll(DrawingUtilitiesClass* duc);

	};

}

