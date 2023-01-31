#pragma once

#include <map>

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
			float stiffness, float damping, float restLength);
		~Spring();

		void applyForce();

		Rigidbody* b0;
		Rigidbody* b1;

		float damping;
		float stiffness;
		float restLength;

		int id;

		static void drawAll(DrawingUtilitiesClass* duc);

	};

}

