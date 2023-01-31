#pragma once

#include <cmath>
#include <iostream>

#include "Mat3.h"

namespace GamePhysics {

	class Collider;
	class Rigidbody;

	class Transform
	{
	public:

		Transform();

		Vec3 position;
		Quat rotation;

		Mat4 transformation;
		Mat4 inverseTransformation;

		void update();
		void setTransformation(const Mat4& T);
	};
}
