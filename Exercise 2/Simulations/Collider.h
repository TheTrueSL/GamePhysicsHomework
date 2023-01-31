#pragma once

#include "Rigidbody.h"

#include <vector>

class DrawingUtilitiesClass;

namespace GamePhysics {
	
	class ContactInfo {
	public:
		bool isValid = false;
		Vec3 point;
		Vec3 normal;
		float depth = 0;
	};

	class Collider
	{
	public:
		enum class ColliderType {
			Sphere,
			Box
		};

	public:
		static std::map<int, Collider*> dict;
		static int id_count;

		static ContactInfo collisionSphereSphere(Collider* s0, Collider* s1);
		static ContactInfo collisionBoxSphere(Collider* b, Collider* s);
		static ContactInfo collisionBoxBox(Collider* b0, Collider* b1);

		Collider(Transform*, Rigidbody* body=nullptr, int layer=0b10, int filter=0b11);
		~Collider();

		void setBox(Vec3 size);
		void setSphere(float radius);

		ContactInfo collisionTest(Collider* other);
		ContactInfo collisionTest(Plane* plane);

		void draw(DrawingUtilitiesClass* duc);
		void drawShadow(DrawingUtilitiesClass* duc, float y, const Vec3& dir);

		bool drawAxis = false;
		bool isTrigger = false;

		ColliderType type;
		Vec3 size;
		Vec3 halfSize;

		Transform* transform;
		Rigidbody* rigidbody;

		bool passForceToBodyAndUsePointDynamic;
		Vec3 lastPosition;
		Vec3 pointBasedVelocity;

		int filter = 0b11111111; // 0b01 : contact with ground
		int layer  = 0b10; // 0b01 : contact with ground
							// 0b10 : default
		void update();
		// SAT test properties
		std::vector<Vec3> corners;
		std::vector<Vec3> worldCorners;
		Vec3 basisAxis[3];

		// coarse collision detection
		Vec3 coarseCenter;
		float coarseRadius;

		int id;
	};

}
