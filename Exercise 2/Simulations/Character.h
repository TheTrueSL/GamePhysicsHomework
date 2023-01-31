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

		Vec3 springPivot;
		Vec3 springs[4];
		bool enableSprings[4];
		float springPower;

		float targetLLength;
		float targetRLength;

		float lLength;
		float rLength;

		GameObject body; // root transform
		Mat3 I0;

		GameObject head;
		
		GameObject lUp;
		GameObject rUp;
		GameObject lLow;
		GameObject rLow;

		GameObject lFoot;
		GameObject rFoot;

		Vec3 lOffset;
		float lq1, lq2, lq3;
		float ll1, ll2;

		Vec3 rOffset;
		float rq1, rq2, rq3;
		float rl1, rl2;

		Vec3 headOffset;

		void updateInertiaByPointMasses();

		Vec3 pFoot(float q1, float q2, float q3, float l1, float l2);
		void JFoot(float q1, float q2, float q3, float l1, float l2,
			Vec3& row1, Vec3& row2, Vec3& row3);
		void JLow(float q1, float q2, float q3, float l1, float l2);
		void JUp(float q1, float q2, float l1);

		void LegIK(float& q1, float& q2, float& q3, float& l1, float& l2, const Vec3& localTarget);

		void LegTransformations(float q1, float q2, float q3, float l1, float l2,
			Mat4& Up, Mat4& Low, Mat4& Foot);

		void updateKinematics(bool foot);// update body parts and 
							//assign temp velocity for collisions with other bodies

	};

}