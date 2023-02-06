#pragma once

#include "Transform.h"

#include <map>

namespace GamePhysics {
	class Plane {
	public:
		Vec3 normal;
		float offset;
		float sfriction;
		float dfriction;
	};
	class Rigidbody
	{
	public:
		static int id_count;
		static std::map<int, Rigidbody*> dict;

	public:
		Rigidbody(Transform* transform);
		~Rigidbody();

		void setSphereInertia(float m, float r);
		void setBoxInertia(float m, Vec3 size);

		void addForce(Vec3 force, Vec3 point);
		void update();
		void init();

		Vec3 velocity;
		Vec3 angularVelocity;
		Vec3 angularMomentum;

		Vec3 force;
		Vec3 torque;

		Transform* transform;

		Mat3 inertia;
		Mat3 inverseInertia;

		float mass;
		float inverseMass;

		float friction;
		bool isFixed;
		bool fixRotation;
		bool fixPosition;
		bool useGravity;
		Vec3 CoM;

		Mat3 worldInvInertia;
		Vec3 worldCoM;

		int id;

	};
}
