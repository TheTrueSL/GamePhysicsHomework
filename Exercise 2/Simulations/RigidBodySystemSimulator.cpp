#include "RigidBodySystemSimulator.h"

#include<random>
#include<time.h>

using namespace GamePhysics;

namespace {
	const Vec3 ZERO_VEC(0, 0, 0);
	const float EPS = 1e-6;
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	gameManager = new GameManager(this, &this->player);
	DUC = nullptr;
	reset();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	// custom case name, split with comma","
	// notifyCaseChanged(int index) will be called when case is changed by user.
	return "Test";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_x", TW_TYPE_DOUBLE, &this->_gravity.x, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_y", TW_TYPE_DOUBLE, &this->_gravity.y, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_z", TW_TYPE_DOUBLE, &this->_gravity.z, "");
	TwAddVarRW(DUC->g_pTweakBar, "LinearDamping", TW_TYPE_FLOAT, &this->_linearDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "AngularDamping", TW_TYPE_FLOAT, &this->_angularDamping, "");

}

void RigidBodySystemSimulator::reset()
{
	_passedTime = 0;

	deleteObjects();

	_isPressed = false;
	_pressedTimer = clock();

	_linearDamping = 0;
	_angularDamping = 0;

	// UI Attributes
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	const Vec3 white = Vec3(1., 1., 1.);
	const Vec3 gray = white * 0.35;
	const Vec3 red = Vec3(1., 0., 0.);

	Vec3 dir(0.01, -2.0, 0.01);
	float y_plane = -0.5;
	normalize(dir);
	DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 10, white);
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->draw(DUC);
		objects[i]->collider->drawShadow(DUC, y_plane, dir);
	}
	for (int i = 0; i < fixedObjects.size(); i++) {
		fixedObjects[i]->draw(DUC);
		fixedObjects[i]->collider->drawShadow(DUC, y_plane, dir);
	}
	for (int i = 0; i < customObjects.size(); i++) {
		customObjects[i]->onDraw(DUC);
		customObjects[i]->collider->drawShadow(DUC, y_plane, dir);
		customObjects[i]->onFrameUpdate();
	}
	for (int i = 0; i < customFixedObjects.size(); i++) {
		customFixedObjects[i]->onDraw(DUC);
		customFixedObjects[i]->collider->drawShadow(DUC, y_plane, dir);
		customFixedObjects[i]->onFrameUpdate();
	}

	gameManager->onFrameUpdate(DUC, pd3dImmediateContext);

	player.draw(DUC);

	DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 0, white);

	Spring::drawAll(DUC);
	
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	loadTestSetup();
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	return;

	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if ((mouseDiff.x) != 0 || (mouseDiff.y) != 0)
	{
		Mat4 View = Mat4(DUC->g_camera.GetViewMatrix());
		Mat4 ViewInv = View.inverse();

		Mat4 Proj = Mat4(DUC->g_camera.GetProjMatrix());
		Mat4 ProjInv = Proj.inverse();

		float mx = -1 + 2 * (float)m_trackmouse.x / DUC->g_windowSize[0];
		float my = 1 - 2 * (float)m_trackmouse.y / DUC->g_windowSize[1];

		Vec3 v0 = Vec3(mx, my, 0);
		Vec3 v1 = Vec3(mx, my, 1);
		XMVECTOR view0 = XMVector4Transform(
			v0.toDirectXVector(), ProjInv.toDirectXMatrix());
		XMVECTOR w_vector0 = XMVectorSplatW(view0);
		view0 = XMVectorDivide(view0, w_vector0);

		XMVECTOR view1 = XMVector4Transform(
			v1.toDirectXVector(), ProjInv.toDirectXMatrix());
		XMVECTOR w_vector1 = XMVectorSplatW(view1);
		view1 = XMVectorDivide(view1, w_vector1);

		/*Vec3 viewDragOrigin = View.transformVector(_dragOrigin);

		Vec3 ray = (view1 - view0);
		GamePhysics::normalize(ray);

		Vec3 viewDragPoint = ray * norm(viewDragOrigin);*/
		//_dragMousePoint = ViewInv.transformVector(viewDragPoint);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	onStepStart(timeStep);
	integrateExplicitEuler(timeStep);
	onStepEnd(timeStep);
	for (int i = 0; i < customObjects.size(); i++) {
		customObjects[i]->onPhysicsUpdate(timeStep);
	}
	gameManager->onPhysicUpdate(timeStep);

}

void RigidBodySystemSimulator::onKeyboardPressed(unsigned int key)
{
	player.onKeyPressed(key);
}

void RigidBodySystemSimulator::onKeyboardReleased(unsigned int key)
{
	player.onKeyReleased(key);
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	if (!_isPressed) {

		time_t hoverTime = _pressedTimer;
		_pressedTimer = clock();
		hoverTime = _pressedTimer - hoverTime;
		if (hoverTime < 250) { // 250ms
			onMouseDouble(x, y);
			_pressedTimer = _pressedTimer - 250;
		}
		else {
			onMouseDown(x, y);
		}
	}
	else {
		Vec3 ro, rd;
		screen2Ray(x, y, ro, rd);
		player.onMouseMove(x, y, ro, rd);
	}

	_isPressed = true;

	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	if (_isPressed) {
		player.onMouseReleased(x, y);
	}
	_isPressed = false;

	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouseDown(int x, int y)
{
	Vec3 ro, rd;
	screen2Ray(x, y, ro, rd);
	player.onMousePressed(x, y, ro, rd);
}

void RigidBodySystemSimulator::onMouseDouble(int x, int y)
{
	//selectDragPoint(x, y, dragCollider);
}

void RigidBodySystemSimulator::screen2Ray(int x, int y, Vec3& outO, Vec3& outD)
{
	// find closest point mass
	float mx = (float)x / DUC->g_windowSize[0] * 2 - 1;
	float my = 1 - (float)y / DUC->g_windowSize[1] * 2;

	XMMATRIX viewProj = DUC->g_camera.GetViewMatrix() *
		DUC->g_camera.GetProjMatrix();

	Mat4 Proj = Mat4(DUC->g_camera.GetProjMatrix());
	Mat4 ProjInv = Proj.inverse();
	Mat4 View = Mat4(DUC->g_camera.GetViewMatrix());
	Mat4 ViewInv = View.inverse();

	Vec3 v0 = Vec3(mx, my, 0);
	Vec3 v1 = Vec3(mx, my, 1);
	XMVECTOR view0 = XMVector4Transform(
		v0.toDirectXVector(), ProjInv.toDirectXMatrix());
	XMVECTOR w_vector0 = XMVectorSplatW(view0);
	view0 = XMVectorDivide(view0, w_vector0);

	XMVECTOR view1 = XMVector4Transform(
		v1.toDirectXVector(), ProjInv.toDirectXMatrix());
	XMVECTOR w_vector1 = XMVectorSplatW(view1);
	view1 = XMVectorDivide(view1, w_vector1);

	Vec3 rayOrigin = ViewInv.transformVector(ZERO_VEC);
	Vec3 rayDir = (view1 - view0);
	GamePhysics::normalize(rayDir);
	rayDir = ViewInv.transformVectorNormal(rayDir);

	outO = rayOrigin;
	outD = rayDir;
}

void RigidBodySystemSimulator::loadTestSetup() {
	reset();

	_linearDamping = 0.025;
	_angularDamping = 1e-4;

	Vec3 gravity(0, -2.2, 0);

	_gravity = gravity;

	gameManager->onStart();

	// box 0
	/*{
		Transform* transform = new Transform();
		transform->position = Vec3(0, 100.25, 6.5);
		transform->rotation = Quat(0.5, 0, 0.25, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		rigidbody->setBoxInertia(8, Vec3(0.4, 0.5, 0.4));
		rigidbody->angularMomentum = Vec3(0, 1, 0);
		rigidbody->friction = 1.1;
		Collider* collider = new Collider(transform, rigidbody);
		collider->setBox(Vec3(0.4, 0.5, 0.4));
		objects.push_back(new GameObject(transform, rigidbody, collider));

	}*/


	//Generate net
	{
	}


	updateTransformations();
}

void RigidBodySystemSimulator::bindCustomObject(CustomObject* cobj)
{
	if (cobj == nullptr)
		return;
	if (cobj->rigidbody == nullptr)
		return;

	colliderMap.insert(std::make_pair(cobj->collider->id, cobj));
	if (cobj->rigidbody->isFixed) {
		customFixedObjects.push_back(cobj);
	}
	else {
		customObjects.push_back(cobj);
	}
}

void RigidBodySystemSimulator::bindGameObject(GameObject* gobj)
{
	if (gobj == nullptr)
		return;
	if (gobj->rigidbody == nullptr)
		return;
	if (gobj->rigidbody->isFixed) {
		fixedObjects.push_back(gobj);
	}
	else {
		objects.push_back(gobj);
	}
}

void RigidBodySystemSimulator::bindSpring(Spring* s)
{
	springs.push_back(s);
}

void RigidBodySystemSimulator::collisionResolve(
	const float& deltaTime)
{
	collisionRigidBodies(deltaTime);
	collisionPLane(deltaTime, Vec3(0, 1, 0), -0.5, 1);
}

void RigidBodySystemSimulator::collisionPLane(
	const float& deltaTime,
	const Vec3& n, const float offset, const float friction)
{
	const float dynFriction = friction * 0.8;
	const float possoft = 0.05;
	const float velsoft = 0.5;
	const float c = 0.05;

	Plane plane;
	plane.normal = n;
	plane.offset = offset;
	plane.sfriction = friction;
	plane.dfriction = dynFriction;

	for (auto cit = Collider::dict.begin(); cit != Collider::dict.end(); ++cit)
	{
		Collider* collider = cit->second;

		if ((collider->filter & 1) == 0 || collider->rigidbody->isFixed)
			continue;

		ContactInfo hit = collider->collisionTest(&plane);

		if (hit.isValid) {
			Rigidbody* rigidbody = collider->rigidbody;
			if (collider->isTrigger) {
				const auto it = colliderMap.find(collider->id);
				if (it != colliderMap.end()) {
					it->second->onCollisionEnter(nullptr);
				}
				continue;
			}

			if (rigidbody != nullptr && !rigidbody->isFixed) {
				float iM0 = rigidbody->inverseMass;
				Mat3 iI0;
				iI0.setZero();
				if (!rigidbody->fixRotation) {
					iI0 = rigidbody->inverseInertia;
				}

				const Vec3 x0 = (hit.point - rigidbody->worldCoM);
				const Vec3& n = hit.normal;

				Vec3 vrel = rigidbody->velocity + 
					cross(rigidbody->angularVelocity, x0)
					- (dot(_gravity, n) * deltaTime) * n;

				float vdot = dot(vrel, n);

				if (vdot < 0) {
					float J = (-(1 + c) * vdot) /
						(iM0 + dot(
							cross(iI0.dot(cross(x0, n)), x0), n));
					const Vec3 Jn = J * n;
					rigidbody->velocity += Jn * iM0 * velsoft;
					rigidbody->angularMomentum += cross(x0, Jn) * velsoft;
				}

				rigidbody->transform->position += n * hit.depth * possoft;

				Vec3 vTangent = vrel - (n * vdot);
				float fNormal = dot(rigidbody->force, n);
				if (fNormal < 0) {
					if (norm(vTangent) > 1e-2) {
						Vec3 dynamicFriction = -(dynFriction + rigidbody->friction) * 0.5f
							* (-fNormal) * vTangent;
						rigidbody->addForce(dynamicFriction, hit.point);
					}
					else{
						Vec3 fTangent = rigidbody->force - (n * fNormal);
						float fTangNorm = norm(fTangent);
						if (fTangNorm > 0) {
							// static
							Vec3 staticFriction = min(-fTangent,
								-(friction + rigidbody->friction) * 0.5f * (-fNormal) * fTangent / fTangNorm);
							rigidbody->addForce(staticFriction, hit.point);
						}
					}
				}
			}
		}
	}
}

void RigidBodySystemSimulator::collisionRigidBodies(const float& deltaTime)
{
	const float bounciness = 0.0;
	const float possoft = 0.03;
	const float velsoft = 0.5;
	for (auto cit0 = Collider::dict.begin(); cit0 != Collider::dict.end(); ++cit0)
	{
		Collider* collider0 = cit0->second;
		Rigidbody* b0 = collider0->rigidbody;

		for (auto cit1 = std::next(cit0); cit1 != Collider::dict.end(); ++cit1)
		{
			Collider* collider1 = cit1->second;
			Rigidbody* b1 = collider1->rigidbody; 

			if ((collider0->filter & collider1->layer) == 0 && ((collider1->filter & collider0->layer) == 0))
				continue;

			if ((b0 == nullptr || b0->isFixed)
				&& (b1 == nullptr || b1->isFixed))
				continue;

			ContactInfo hit = collider0->collisionTest(collider1);

			if (hit.isValid) {
				if (collider0->isTrigger) {
					const auto it = colliderMap.find(collider0->id);
					if (it != colliderMap.end()) {
						it->second->onCollisionEnter(collider1);
					}
					continue;
				}
				else if (collider1->isTrigger) {
					const auto it = colliderMap.find(collider1->id);
					if (it != colliderMap.end()) {
						it->second->onCollisionEnter(collider0);
					}
					continue;
				}

				float iM0 = 0;
				float iM1 = 0;
				Mat3 iI0;
				Mat3 iI1;
				Vec3 v0(0, 0, 0);
				Vec3 v1(0, 0, 0);
				const Vec3 x0 = (hit.point - b0->worldCoM);
				const Vec3 x1 = (hit.point - b1->worldCoM);
				const Vec3& n = hit.normal;

				if (!b0->isFixed) {
					iM0 = b0->inverseMass;
					iI0 = b0->worldInvInertia;
				}

				if (collider0->passForceToBodyAndUsePointDynamic) {
					v0 = collider0->pointBasedVelocity;
				}
				else {
					v0 = b0->velocity +
						cross(b0->angularVelocity, x0);
				}

				if (!b1->isFixed) {
					iM1 = b1->inverseMass;
					iI1 = b1->worldInvInertia;
				}

				if (collider1->passForceToBodyAndUsePointDynamic) {
					v1 = collider1->pointBasedVelocity;
				}
				else {
					v1 = b1->velocity +
						cross(b1->angularVelocity, x1);
				}

				Vec3 vrel = v0 - v1;
				float vdot = dot(vrel, n);
				const float c = bounciness;
				float J = (-(1 + c) * vdot) /
					(iM0 + iM1 + dot(
						cross(iI0.dot(cross(x0, n)), x0) +
						cross(iI1.dot(cross(x1, n)), x1), n));
				
				if (vdot < 0) {
					const Vec3 Jn = J * n;

					if (!b0->isFixed) {
						float w0 = 1;
						if (!b1->isFixed) {
							w0 = (b1->mass / (b0->mass + b1->mass));
						}
						b0->transform->position += w0 * hit.depth * n * possoft;
						b0->velocity += Jn * iM0 * velsoft;
						b0->angularMomentum += cross(x0, Jn) * velsoft;
					}
					if (!b1->isFixed) {
						float w1 = 1;
						if (!b0->isFixed) {
							w1 = (b0->mass / (b0->mass + b1->mass));
						}
						b1->transform->position -= w1 * hit.depth * n * possoft;
						b1->velocity += -Jn * iM1 * velsoft;
						b1->angularMomentum += cross(x1, -Jn) * velsoft;
					}

					Vec3 vTangent = vrel - (n * vdot);
					float fNormal = dot(b0->force - b1->force, n);
					if (fNormal < 0) {
						if (norm(vTangent) > 1e-2) {
							Vec3 dynamicFriction = -(b0->friction + b1->friction) * 0.8f * 0.5f
								* (-fNormal) * vTangent;
							b0->addForce(dynamicFriction, hit.point);
							b1->addForce(-dynamicFriction, hit.point);
						}
						else {
							Vec3 fTangent = (b0->force - b1->force) - (n * fNormal);
							float fTangNorm = norm(fTangent);
							if (fTangNorm > 0) {
								// static
								Vec3 staticFriction = min(-fTangent,
									-(b0->friction + b1->friction) * 0.5f * (-fNormal) * fTangent / fTangNorm);
								b0->addForce(staticFriction, hit.point);
								b1->addForce(-staticFriction, hit.point);
							}
						}
					}
				}
			}
		}
	}
}

void RigidBodySystemSimulator::updateTransformations()
{
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->update();
	}
	for (int i = 0; i < fixedObjects.size(); i++) {
		fixedObjects[i]->update();
	}
	for (int i = 0; i < customObjects.size(); i++) {
		customObjects[i]->update();
	}
	for (int i = 0; i < customFixedObjects.size(); i++) {
		customFixedObjects[i]->update();
	}
	player.updateTransformations();
}

void RigidBodySystemSimulator::computeSpringForce()
{
	for (auto sit = Spring::dict.begin(); sit != Spring::dict.end(); ++sit)
	{
		sit->second->applyForce();
	}
}

void RigidBodySystemSimulator::applyDamping(const float& timeStep)
{
	// damping forces
	for (auto rit = Rigidbody::dict.begin(); rit != Rigidbody::dict.end(); ++rit)
	{
		Rigidbody* rigidbody = rit->second;
		if (rigidbody->isFixed)
			continue;

		rigidbody->velocity +=
			timeStep * (-_linearDamping * rigidbody->velocity / rigidbody->mass);
		rigidbody->angularMomentum +=
			timeStep * -_angularDamping * rigidbody->angularVelocity;
	}
}

void RigidBodySystemSimulator::updatePosition(const float& timeStep)
{
	for (auto rit = Rigidbody::dict.begin(); rit != Rigidbody::dict.end(); ++rit)
	{
		Rigidbody* rigidbody = rit->second;
		Transform* transform = rigidbody->transform;

		if (rigidbody->isFixed) {
			
		}
		else {
			transform->position +=
				rigidbody->velocity * timeStep;
			
			if (!rigidbody->fixRotation) {
				const Vec3& vw = rigidbody->angularVelocity;
				Quat w(vw.x, vw.y, vw.z, 0);
				transform->rotation += (timeStep * 0.5) * w * transform->rotation;
				float quatnorm = transform->rotation.norm();
				if (quatnorm == 0)
				{
					transform->rotation = Quat(0, 0, 0, 1);
				}
				else {
					transform->rotation /= quatnorm;
				}
			}
		}
	}
}

void RigidBodySystemSimulator::updateVelocity(
	const float& timeStep)
{
	for (auto rit = Rigidbody::dict.begin(); rit != Rigidbody::dict.end(); ++rit)
	{
		Rigidbody* rigidbody = rit->second;
		Transform* transform = rigidbody->transform;

		if (rigidbody->isFixed) {

		}
		else {
			rigidbody->velocity +=
				timeStep * (rigidbody->force / rigidbody->mass);
			if (rigidbody->fixRotation) {
				rigidbody->angularMomentum = Vec3(0,0,0);
				rigidbody->angularVelocity = Vec3(0,0,0);
			} else {
				rigidbody->angularMomentum += 
					timeStep * rigidbody->torque;
				rigidbody->angularVelocity = 
					rigidbody->worldInvInertia.dot(rigidbody->angularMomentum);
			}
		}
	}
}

void RigidBodySystemSimulator::deleteObjects()
{
	colliderMap.clear();
	for (auto& o : objects) {
		delete o;
	}
	objects.clear();
	for (auto& o : customObjects) {
		delete o;
	}
	customObjects.clear();
	for (auto& o : fixedObjects) {
		delete o;
	}
	fixedObjects.clear();
	for (auto& o : customFixedObjects) {
		delete o;
	}
	customFixedObjects.clear();
	for (auto& o : springs) {
		delete o;
	}
	springs.clear();
}

void RigidBodySystemSimulator::onStepStart(const float dt)
{
	for (auto cit = Collider::dict.begin(); cit != Collider::dict.end(); ++cit)
	{
		Collider* collider = cit->second;
		collider->lastPosition = collider->transform->position;
	}
}

void RigidBodySystemSimulator::onStepEnd(const float dt)
{
	for (auto cit = Collider::dict.begin(); cit != Collider::dict.end(); ++cit)
	{
		Collider* collider = cit->second;
		collider->pointBasedVelocity = (collider->transform->position - collider->lastPosition) / dt;
	}
}

void RigidBodySystemSimulator::integrateExplicitEuler(const float& timeStep)
{
	applyGravity(_gravity);
	applyDamping(timeStep);

	player.onUpdate(timeStep);

	computeSpringForce();

	collisionResolve(timeStep);

	updatePosition(timeStep);
	updateTransformations();
	updateVelocity(timeStep);

	clearForce();
}

void RigidBodySystemSimulator::clearForce()
{
	for (auto rit = Rigidbody::dict.begin(); rit != Rigidbody::dict.end(); ++rit)
	{
		Rigidbody* rigidbody = rit->second;
		rigidbody->force = ZERO_VEC;
		rigidbody->torque = ZERO_VEC;
	}
}

void RigidBodySystemSimulator::applyGravity(const Vec3& g)
{
	for (auto rit = Rigidbody::dict.begin(); rit != Rigidbody::dict.end(); ++rit)
	{
		Rigidbody* rigidbody = rit->second;
		if (!rigidbody->isFixed && rigidbody->useGravity) {
			rigidbody->force += rigidbody->mass * g;
		}
	}
}
