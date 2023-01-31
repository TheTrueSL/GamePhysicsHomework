#include "Character.h"

#define KEY_UP 38
#define KEY_DOWN 40
#define KEY_LEFT 37
#define KEY_RIGHT 39

using namespace GamePhysics;

GamePhysics::Character::Character()
{
	// local offsets
	headOffset = Vec3(0, 0.21, 0);

	lOffset = Vec3(-0.08, -0.11, 0);
	lq1 = 0;
	lq2 = 0;
	lq3 = 0;
	ll1 = 0.2;
	ll2 = 0.2;

	rOffset = Vec3(0.08, -0.11, 0);
	rq1 = 0;
	rq2 = 0;
	rq3 = 0;
	rl1 = 0.2;
	rl2 = 0.2;

	buildModel();

	// support force
	springPivot = Vec3(0, 1, 0);
	springs[0] = Vec3(1, 0, 0);
	springs[1] = Vec3(0, 0, 1);
	springs[2] = Vec3(-1, 0, 0);
	springs[3] = Vec3(0, 0, -1);
	enableSprings[0] = false;
	enableSprings[1] = false;
	enableSprings[2] = false;
	enableSprings[3] = false;
	springPower = 1;
}

GamePhysics::Character::~Character()
{
}

void GamePhysics::Character::onUpdate(const float dt)
{
	updateInertiaByPointMasses();
	const float speed = 11.0;
	const float lerp_t = std::min(dt * speed, 1.0f);
	const float length_v = 0.65;

	lLength = lLength * (1 - lerp_t) + targetLLength * lerp_t;
	rLength = rLength * (1 - lerp_t) + targetRLength * lerp_t;

	const float stiffness = 2.5;
	const float ground = -0.5;

	Vec3 lAxis(0, -1, 0);
	Vec3 rAxis(0, -1, 0);

	Vec3 wlAxis = body.transform->transformation.transformVectorNormal(lAxis);
	Vec3 wrAxis = body.transform->transformation.transformVectorNormal(rAxis);

	bool lTouchground = false;
	bool rTouchground = false;

	Vec3 totalLinearImpact;
	Vec3 totalAngularImpact;

	const float ground_soft = 0.5;
	const float ground_friction = 8.0;

	// hook's law
	{
		Vec3 lLegRoot = body.transform->transformation.transformVector(lOffset);
		lFoot.transform->position = lLegRoot + abs(dot(lFoot.transform->position - lLegRoot, wlAxis)) * wlAxis;

		Vec3 v0 = lFoot.transform->position - lLegRoot;
		float length = normalize(v0);

		const Vec3 x0 = (lFoot.transform->position - body.rigidbody->worldCoM);
		const Vec3 n(0, 1, 0);

		Vec3 vrel = body.rigidbody->velocity +
			cross(body.rigidbody->angularVelocity, x0);

		//lFootV += dt * f0;
		lFoot.transform->position += -(length - lLength) * v0 * length_v;
		lFoot.rigidbody->velocity = vrel - (length - lLength) * v0 * dt;

		if (lFoot.transform->position.y <= ground) {
			lFoot.transform->position += (lFoot.transform->position.y - ground) * 
				wlAxis / (abs(wlAxis.y) + 1e-8) * ground_soft;

			float vdot = dot(vrel, n);

			if (vdot < 0) {
				const float c = 0;
				float J = (-(1) * vdot) /
					(body.rigidbody->inverseMass + dot(
						cross(body.rigidbody->inverseInertia.dot(cross(x0, n)), x0), n));
				const Vec3 Jn = J * n;

				totalLinearImpact += Jn * body.rigidbody->inverseMass * ground_soft;
				totalAngularImpact += cross(x0, Jn) * ground_soft;
			}

			Vec3 vTangent = vrel - (n * vdot);
			float fNormal = dot(body.rigidbody->force, n);
			if (fNormal < 0) {
				Vec3 dynamicFriction = -ground_friction * (-fNormal) * vTangent;
				body.rigidbody->addForce(dynamicFriction, lFoot.transform->position);
				lTouchground = true;
			}
		}
		// forces
		Vec3 f0 = -(length - lLength) * v0;
		if (lTouchground) {
			body.rigidbody->addForce(-(stiffness) * v0 * body.rigidbody->mass, lFoot.transform->position);
		}

		Vec3 local = body.transform->inverseTransformation.transformVector(lFoot.transform->position);
		local = local - lOffset;
		LegIK(lq1, lq2, lq3, ll1, ll2, local);
	}

	{
		Vec3 rLegRoot = body.transform->transformation.transformVector(rOffset);
		
		rFoot.transform->position = rLegRoot + abs(dot(rFoot.transform->position - rLegRoot, wrAxis)) * wrAxis;

		Vec3 v0 = rFoot.transform->position - rLegRoot;
		float length = normalize(v0);
		const Vec3 x0 = (rFoot.transform->position - body.rigidbody->worldCoM);
		const Vec3 n(0, 1, 0);
		Vec3 vrel = body.rigidbody->velocity +
			cross(body.rigidbody->angularVelocity, x0);

		rFoot.transform->position += -(length - rLength) * v0 * length_v;
		rFoot.rigidbody->velocity = vrel -(length - rLength) * v0 * dt;
		//rFootV += dt * f0;
		//rFoot.transform->position += rFootV * dt;
		if (rFoot.transform->position.y <= ground) {
			rFoot.transform->position += (rFoot.transform->position.y - ground) 
				* wrAxis / (abs(wrAxis.y) + 1e-8) * ground_soft;
			
			float vdot = dot(vrel, n);

			if (vdot < 0) {
				const float c = 0;
				float J = (-(1 + c) * vdot) /
					(body.rigidbody->inverseMass + dot(
						cross(body.rigidbody->inverseInertia.dot(cross(x0, n)), x0), n));
				const Vec3 Jn = J * n;

				totalLinearImpact += Jn * body.rigidbody->inverseMass * ground_soft;
				totalAngularImpact += cross(x0, Jn) * ground_soft;
			}

			Vec3 vTangent = vrel - (n * vdot);
			float fNormal = dot(body.rigidbody->force, n);
			if (fNormal < 0) {
				Vec3 dynamicFriction = -ground_friction * (-fNormal) * vTangent;
				body.rigidbody->addForce(dynamicFriction, rFoot.transform->position);
			}
			rTouchground = true;
		}

		// forces
		Vec3 f0 = -(length - rLength) * v0;
		if (rTouchground) {
			//body.rigidbody->velocity += -stiffness * v0 * dt;
			body.rigidbody->addForce(-(stiffness) * v0 * body.rigidbody->mass, rFoot.transform->position);
		}

		Vec3 local = body.transform->inverseTransformation.transformVector(rFoot.transform->position);
		local = local - rOffset;
		LegIK(rq1, rq2, rq3, rl1, rl2, local);
	}

	body.rigidbody->velocity += totalLinearImpact * 0.5f;
	body.rigidbody->angularMomentum += totalAngularImpact * 0.5f;

	updateKinematics(true);

}

void GamePhysics::Character::draw(DrawingUtilitiesClass* duc)
{
	Vec3 dir(0.01, -2.0, 0.01);
	float y_plane = -0.5;
	normalize(dir);

	body.draw(duc);
	body.collider->drawShadow(duc, y_plane, dir);
	head.draw(duc);
	head.collider->drawShadow(duc, y_plane, dir);

	lUp.draw(duc);
	rUp.draw(duc);
	lUp.collider->drawShadow(duc, y_plane, dir);
	rUp.collider->drawShadow(duc, y_plane, dir);

	lLow.draw(duc);
	rLow.draw(duc);
	lLow.collider->drawShadow(duc, y_plane, dir);
	rLow.collider->drawShadow(duc, y_plane, dir);

	//lFoot.draw(duc);
	//rFoot.draw(duc);
}

void GamePhysics::Character::init(Vec3 positon)
{
	lq1 = 0;
	lq2 = 0;
	lq3 = 0;

	rq1 = 0;
	rq2 = 0;
	rq3 = 0;

	lLength = (ll1 + ll2) * 0.85;
	rLength = (rl1 + rl2) * 0.85;
	targetLLength = lLength;
	targetRLength = rLength;

	body.transform->position = positon;
	body.transform->rotation = Quat(0, 0, 0, 1);
	body.rigidbody->init();
	body.update();

	head.rigidbody->init();
	lUp.rigidbody->init();
	lLow.rigidbody->init();
	lFoot.rigidbody->init();
	rUp.rigidbody->init();
	rLow.rigidbody->init();
	rFoot.rigidbody->init();

	updateKinematics(true);
}

void GamePhysics::Character::buildModel()
{
	const float leg_thickness = 0.032;
	const float foot_size = 0.03;
	// root
	Transform* rootTransform = new Transform();
	Rigidbody* rootRigidbody = new Rigidbody(rootTransform);
	rootRigidbody->friction = 0.5;
	rootRigidbody->isFixed = false;
	rootRigidbody->setBoxInertia(4.0, Vec3(0.2, 0.4, 0.2));
	I0 = rootRigidbody->inertia;
	{
		Collider* collider = new Collider(rootTransform, rootRigidbody);
		collider->layer = 0b100;
		collider->filter = 0b11;
		collider->setBox(Vec3(0.2, 0.2, 0.15));
		body.regist(rootTransform, rootRigidbody, collider);
	}

	{
		Transform* transform = new Transform();

		Collider* collider = new Collider(transform, rootRigidbody);
		collider->layer = 0b100;
		collider->filter = 0b11;
		collider->setBox(Vec3(0.195, 0.195, 0.15));
		head.regist(transform, rootRigidbody, collider);
	}

	{
		Transform* transform = new Transform();

		Collider* collider = new Collider(transform, rootRigidbody);
		//collider->passForceToBodyAndUsePointDynamic = true;
		collider->layer = 0b100;
		collider->filter = 0b11; 
		collider->setBox(Vec3(leg_thickness, ll1 * 0.9, leg_thickness));
		lUp.regist(transform, rootRigidbody, collider);
	}

	{
		Transform* transform = new Transform();

		Collider* collider = new Collider(transform, rootRigidbody);
		//collider->passForceToBodyAndUsePointDynamic = true;
		collider->layer = 0b100;
		collider->filter = 0b11;
		collider->setBox(Vec3(leg_thickness, rl1 * 0.9, leg_thickness));
		rUp.regist(transform, rootRigidbody, collider);
	}

	{
		Transform* transform = new Transform();
		//rigidbody->setBoxInertia(1, Vec3(1, 1, 1));

		Collider* collider = new Collider(transform, rootRigidbody);
		//collider->passForceToBodyAndUsePointDynamic = true;
		collider->layer = 0b100;
		collider->filter = 0b10;
		collider->setBox(Vec3(leg_thickness, ll2 * 0.9, leg_thickness));
		lLow.regist(transform, rootRigidbody, collider);
	}
	{
		Transform* transform = new Transform();

		Collider* collider = new Collider(transform, rootRigidbody);
		//collider->passForceToBodyAndUsePointDynamic = true;
		collider->layer = 0b100;
		collider->filter = 0b10;
		collider->setBox(Vec3(leg_thickness, rl2 * 0.9, leg_thickness));
		rLow.regist(transform, rootRigidbody, collider);
	}

	{
		Transform* transform = new Transform();

		Rigidbody* rigidbody = new Rigidbody(transform);
		rigidbody->isFixed = true;
		rigidbody->mass = rootRigidbody->mass * 0.5;

		Collider* collider = new Collider(transform, rigidbody);
		collider->layer = 0b100;
		collider->filter = 0b10;
		collider->setSphere(foot_size);
		lFoot.regist(transform, rigidbody, collider);
	}

	{
		Transform* transform = new Transform();

		Rigidbody* rigidbody = new Rigidbody(transform);
		rigidbody->isFixed = true;
		rigidbody->mass = rootRigidbody->mass * 0.5;

		Collider* collider = new Collider(transform, rigidbody);
		collider->layer = 0b100;
		collider->filter = 0b10;
		collider->setSphere(foot_size);
		rFoot.regist(transform, rigidbody, collider);
	}

	rootRigidbody->mass = rootRigidbody->mass * 2;
}

void GamePhysics::Character::updateInertiaByPointMasses()
{
	Mat3& inertia = body.rigidbody->inertia;
	inertia = I0;

	float Ixx = 0;
	float Iyy = 0;
	float Izz = 0;

	float Ixy = 0;
	float Ixz = 0;
	float Iyz = 0;

	auto addMass = [&](const Vec3& p, const float& m) {
		Ixx += (p.y * p.y + p.z * p.z) * m;
		Iyy += (p.x * p.x + p.z * p.z) * m;
		Izz += (p.y * p.y + p.x * p.x) * m;

		Ixy += (p.x * p.y) * m;
		Ixz += (p.x * p.z) * m;
		Iyz += (p.y * p.z) * m;
	};

	Vec3 lFootLocal = body.transform->inverseTransformation.transformVector(lFoot.transform->position);
	lFootLocal = lFootLocal - lOffset;
	addMass(lFootLocal, lFoot.rigidbody->mass);

	Vec3 rFootLocal = body.transform->inverseTransformation.transformVector(rFoot.transform->position);
	rFootLocal = rFootLocal - rOffset;
	addMass(rFootLocal, rFoot.rigidbody->mass);

	inertia.value[0][0] += Ixx;
	inertia.value[1][1] += Iyy;
	inertia.value[2][2] += Izz;

	inertia.value[0][1] += -Ixy;
	inertia.value[1][0] += -Ixy;

	inertia.value[0][2] += -Ixz;
	inertia.value[2][0] += -Ixz;

	inertia.value[1][2] += -Iyz;
	inertia.value[2][1] += -Iyz;

	body.rigidbody->update();
}

Vec3 GamePhysics::Character::pFoot(float q1, float q2, float q3, float l1, float l2)
{
	const float l3 = -0.01;

	float c1 = std::cos(q1);
	float c2 = std::cos(q2);
	float c3 = std::cos(q3);

	float s1 = std::sin(q1);
	float s2 = std::sin(q2);
	float s3 = std::sin(q3);

	float x = s2 * (l3 * c3 + l2 * s3);
	float y = l2 * (c1 * c3 - c2 * s1 * s3) - l3 * (c1 * s3 + c2 * c3 * s1) + l1 * c1;
	float z = l2 * (c3 * s1 + c1 * c2 * s3) - l3 * (s1 * s3 - c1 * c2 * c3) + l1 * s1;

	return Vec3(x, y, z);
}

void GamePhysics::Character::JFoot(float q1, float q2, float q3, float l1, float l2,
	Vec3& row1, Vec3& row2, Vec3& row3)
{
	const float l3 = -0.01;

	float c1 = std::cos(q1);
	float c2 = std::cos(q2);
	float c3 = std::cos(q3);

	float s1 = std::sin(q1);
	float s2 = std::sin(q2);
	float s3 = std::sin(q3);

	row1 = Vec3(0, c2* (l3 * c3 + l2 * s3), s2* (l2 * c3 - l3 * s3));
	row2 = Vec3(l3 * (s1 * s3 - c1 * c2 * c3) - l2 * (c3 * s1 + c1 * c2 * s3) - l1 * s1, s1 * s2 * (l3 * c3 + l2 * s3), -l2 * (c1 * s3 + c2 * c3 * s1) - l3 * (c1 * c3 - c2 * s1 * s3));
	row3 = Vec3(l2 * (c1 * c3 - c2 * s1 * s3) - l3 * (c1 * s3 + c2 * c3 * s1) + l1 * c1, -c1 * s2 * (l3 * c3 + l2 * s3), -l2 * (s1 * s3 - c1 * c2 * c3) - l3 * (c3 * s1 + c1 * c2 * s3));
}

void GamePhysics::Character::JLow(float q1, float q2, float q3, float l1, float half_l2)
{
}

void GamePhysics::Character::JUp(float q1, float q2, float half_l1)
{
}

void GamePhysics::Character::LegIK(float& q1, float& q2, float& q3, float& l1, float& l2, 
	const Vec3& localTarget)
{
	int maxIteration = 430;
	float step = 0.025f;

	Vec3 target = localTarget;

	target.z -= 0.01;
	float l = norm(target);
	if (l > (l1 + l2)) {
		target *= (l1 + l2) / l;
	}

	Vec3 row1, row2, row3;
	while (maxIteration-- > 0)
	{
		// constraints
		/*if (abs(q2) > M_PI)
		{
			q2 = 0;
		}
		q3 = -abs(q3);*/

		Vec3 dx = target - pFoot(q1, q2, q3, -l1, -l2);
		float d = norm(dx);
		if (d < 1e-5)
		{
			break;
		}

		JFoot(q1, q2, q3, -l1, -l2, row1, row2, row3);

		float dq1 = row1.x * dx.x + row2.x * dx.y + row3.x * dx.z;
		float dq2 = row1.y * dx.x + row2.y * dx.y + row3.y * dx.z;
		float dq3 = row1.z * dx.x + row2.z * dx.y + row3.z * dx.z;

		q1 += step * dq1;
		q2 += step * dq2;
		q3 += step * dq3;
	}
}

void GamePhysics::Character::LegTransformations(float q1, float q2, float q3, float l1, float l2, 
	Mat4& UpT, Mat4& LowT, Mat4& FootT)
{
	Mat4 Rx1 = Mat4(
		1, 0, 0, 0,
		0, std::cos(q1), -sin(q1), 0,
		0, std::sin(q1), std::cos(q1), 0,
		0, 0, 0, 1
	);
	Rx1.transpose();

	Mat4 Ry1 = Mat4(
		cos(q2), 0, sin(q2), 0,
		0, 1, 0, 0,
		-sin(q2), 0, cos(q2), 0,
		0, 0, 0, 1
	);
	Ry1.transpose();

	Mat4 Tyhalf1 = Mat4(
		1, 0, 0, 0,
		0, 1, 0, -l1 * 0.5,
		0, 0, 1, 0,
		0, 0, 0, 1
	);
	Tyhalf1.transpose();

	Mat4 Ty1 = Mat4(
		1, 0, 0, 0,
		0, 1, 0, -l1,
		0, 0, 1, 0,
		0, 0, 0, 1
	);
	Ty1.transpose();

	Mat4 Rx2 = Mat4(
		1, 0, 0, 0,
		0, cos(q3), -sin(q3), 0,
		0, sin(q3), cos(q3), 0,
		0, 0, 0, 1
	);
	Rx2.transpose();

	Mat4 Tyhalf2 = Mat4(
		1, 0, 0, 0,
		0, 1, 0, -l2 * 0.5,
		0, 0, 1, 0,
		0, 0, 0, 1
	);
	Tyhalf2.transpose();

	Mat4 Ty2 = Mat4(
		1, 0, 0, 0,
		0, 1, 0, -l2,
		0, 0, 1, 0,
		0, 0, 0, 1
	);
	Ty2.transpose();

	//UpT = Rx1 * Ry1 * Tyhalf1;
	//LowT = Rx1 * Ry1 * Ty1 * Rx2 * Tyhalf2;
	//FootT = Rx1 * Ry1 * Ty1 * Rx2 * Ty2;

	UpT = Tyhalf1 * Ry1 * Rx1;
	LowT = Tyhalf2 * Rx2 * Ty1 * Ry1 * Rx1;
	FootT = Ty2 * Rx2 * Ty1 * Ry1 * Rx1;
}

void GamePhysics::Character::updateTransformations()
{
	body.update();
	updateKinematics(false);
}

void GamePhysics::Character::onKeyPressed(unsigned int key)
{
	if (key == KEY_LEFT) {
		targetLLength = (ll1 + ll2) * 0.2;
	}
	else if (key == KEY_RIGHT) {
		targetRLength = (rl1 + rl2) * 0.2;
	}
}

void GamePhysics::Character::onKeyReleased(unsigned int key)
{
	if (key == KEY_LEFT) {
		targetLLength = (ll1 + ll2) * 0.95;
	}
	else if (key == KEY_RIGHT) {
		targetRLength = (rl1 + rl2) * 0.95;
	}
}

void GamePhysics::Character::updateKinematics(bool foot)
{
	Mat4 headTrans;
	headTrans.initTranslation(headOffset.x, headOffset.y, headOffset.z);
	head.transform->setTransformation(headTrans * body.transform->transformation);
	head.collider->update();
	head.rigidbody->update();

	Mat4 lUpT;
	Mat4 lLowT;
	Mat4 lFootT;

	Mat4 lLegTrans;
	lLegTrans.initTranslation(lOffset.x, lOffset.y, lOffset.z);
	lLegTrans = lLegTrans * body.transform->transformation;

	LegTransformations(lq1, lq2, lq3, ll1, ll2,
		lUpT, lLowT, lFootT);

	lUp.transform->setTransformation(lUpT * lLegTrans);
	lUp.collider->update();
	lUp.rigidbody->update();

	lLow.transform->setTransformation(lLowT * lLegTrans);
	lLow.collider->update();
	lLow.rigidbody->update();

	if (foot) {
		lFoot.transform->setTransformation(lFootT * lLegTrans);
	}
	lFoot.collider->update();
	lFoot.rigidbody->update();

	Mat4 rUpT;
	Mat4 rLowT;
	Mat4 rFootT;

	Mat4 rLegTrans;
	rLegTrans.initTranslation(rOffset.x, rOffset.y, rOffset.z);
	rLegTrans = rLegTrans * body.transform->transformation;

	LegTransformations(rq1, rq2, rq3, rl1, rl2,
		rUpT, rLowT, rFootT);

	rUp.transform->setTransformation(rUpT * rLegTrans);
	rUp.collider->update();
	rUp.rigidbody->update();
	
	rLow.transform->setTransformation(rLowT * rLegTrans);
	rLow.collider->update();
	rLow.rigidbody->update();
	
	if (foot) {
		rFoot.transform->setTransformation(rFootT * rLegTrans);
	}
	rFoot.collider->update();
	rFoot.rigidbody->update();
}
