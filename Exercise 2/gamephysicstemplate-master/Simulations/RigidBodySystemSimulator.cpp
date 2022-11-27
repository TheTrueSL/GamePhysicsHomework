#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"

#include<random>
#include<time.h>

namespace {
	const Vec3 ZERO_VEC(0, 0, 0);
}

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
	DUC = nullptr;

	_enableCollision = false;
	_enableGraviy = false;
	_enableShoot = false;

	_printSteps = -1;

	reset();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	// custom case name, split with comma","
	// notifyCaseChanged(int index) will be called when case is changed by user.
	return "Demo1(simple 1 step), Demo2(single), Demo3(two body), Demo4(complex)";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator Type",
		"Euler, Leapfrog");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &this->_integrator, "");
	
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &this->_enableGraviy, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_x", TW_TYPE_DOUBLE, &this->_gravity.x, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_y", TW_TYPE_DOUBLE, &this->_gravity.y, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_z", TW_TYPE_DOUBLE, &this->_gravity.z, "");
	TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_BOOLCPP, &this->_enableCollision, "");
	TwAddVarRW(DUC->g_pTweakBar, "Floor", TW_TYPE_BOOLCPP, &this->_enableFloorCollision, "");
	TwAddVarRW(DUC->g_pTweakBar, "Wall", TW_TYPE_BOOLCPP, &this->_enableWallCollision, "");
	TwAddVarRW(DUC->g_pTweakBar, "LinearDamping", TW_TYPE_FLOAT, &this->_linearDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "AngularDamping", TW_TYPE_FLOAT, &this->_angularDamping, "");
	TwAddVarRW(DUC->g_pTweakBar, "ExternalForce", TW_TYPE_FLOAT, &this->_externalSpringForce, "");

	TwAddVarRW(DUC->g_pTweakBar, "ShootMode", TW_TYPE_BOOLCPP, &this->_enableShoot, "");

}

void RigidBodySystemSimulator::reset()
{
	_passedTime = 0;

	_points.clear();
	_springs.clear();
	_rigidbodies.clear();
	_bullets.clear();

	_fillBullet = false;

	_isPressed = false;
	_isDragging = false;
	_pressedTimer = clock();

	_dragIndex = -1;
	_dragType = -1;

	_linearDamping = 0;
	_angularDamping = 0;

	// UI Attributes
	_externalSpringForce = 0;
	m_externalForce = Vec3();
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	const int POINT_SIZE = _points.size();
	const int SPRING_SIZE = _springs.size();
	const int RIGIDBODY_SIZE = _rigidbodies.size();

	const Vec3 white = Vec3(1., 1., 1.);
	const Vec3 gray = white * 0.35;
	const Vec3 red = Vec3(1., 0., 0.);
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = _points[pid];
		if (p._isFixed) {
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, red);
		}
		else {
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, white);
		}
		this->DUC->drawSphere(p._position, p._radius);
	}
	/*DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, white);
	this->DUC->drawSphere(
		_rigidbodies[_dragIndex]._transformation.transformVector(_dragObjectPoint)
		, 0.05);*/
	for (int rid = 0; rid < RIGIDBODY_SIZE; rid++) {
		RigidBody& body = _rigidbodies[rid];
		if (body._isFixed) {
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, red);
		}
		else if(body._isSleep){
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, gray);
		}
		else {
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, white);
		}
		this->DUC->drawRigidBody(body._renderTransformation);
	}

	DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 0, white);

	this->DUC->beginLine();
	for (int sid = 0; sid < SPRING_SIZE; sid++) {
		Spring& s = _springs[sid];
		int pid0 = s._pointID0;
		int pid1 = s._pointID1;
		{
			PointMass& p0 = _points[pid0];
			PointMass& p1 = _points[pid1];
			this->DUC->drawLine(p0._position, white, p1._position, red);
		}
	}
	if (_isDragging) {
		if (_dragType == 0) {
			this->DUC->drawLine(_dragMousePoint, red, _points[_dragIndex]._position, red);
		}
		else if (_dragType == 1) {
			this->DUC->drawLine(_dragMousePoint, red, 
				_rigidbodies[_dragIndex]._transformation.transformVector(_dragObjectPoint)
				, red);
		}
	}
	this->DUC->endLine();
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	std::cout << "Case Changed : " << testCase << std::endl;
	switch (testCase)
	{
	case 0: // Demo1(simple 1 step)
		_printSteps = 2;
		loadSimpleSetup();
		break;
	case 1: // Demo2(simple)
		_printSteps = -1;
		loadSimpleSetup();
		break;
	case 2: // Demo3(two body)
		_printSteps = -1;
		loadTwoBodySetup();
		break;
	case 3: //Demo4(complex)
		_printSteps = -1;
		loadComplexSetup();
		break;
	default:
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
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

		Vec3 viewDragOrigin = View.transformVector(_dragOrigin);

		Vec3 ray = (view1 - view0);
		normalize(ray);

		Vec3 viewDragPoint = ray * norm(viewDragOrigin);
		_dragMousePoint = ViewInv.transformVector(viewDragPoint);

		if (_dragIndex != -1 ) {
			if (_dragType == 0 && _points[_dragIndex]._isFixed) {
				_points[_dragIndex]._position = _dragMousePoint;
			} 
			else if (_dragType == 1 && _rigidbodies[_dragIndex]._isFixed) {
				RigidBody& body = _rigidbodies[_dragIndex];
				body._position = body._transformation.transformVector(
					body._invTransformation.transformVector(_dragMousePoint) - _dragObjectPoint);
				body.updateTransofmration();
			}
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	if (_printSteps == 0)
		return;

	if (_integrator == Integrator::EULER) {
		integrateExplicitEuler(timeStep);
	}
	else if (_integrator == Integrator::LEAPFROG) {
		integrateLeapfrog(timeStep);
	}
	else {
		integrateExplicitEuler(timeStep);
	}

	if (_printSteps > 0) {
		for (int i = 0; i < _points.size(); i++) {
			std::cout << "p " << i << ": " << _points[i]._position << std::endl;
			std::cout << "v " << i << ": " << _points[i]._velocity << std::endl;
		}
		_printSteps -= 1;
	}

	for (int i = _bullets.size() - 1; i >= 0; i--) {
		_bullets[i].lifeTime -= timeStep;
		if (_bullets[i].lifeTime <= 0) {
			_points.erase(_points.begin() + bulletStartIndex + i);
			_bullets.erase(_bullets.begin() + i);
		}
	}
	if (_fillBullet) {
		if (_bullets.size() == 0)
			bulletStartIndex = _points.size();

		_bullets.push_back(Bullet(_points.size(), 2.0f));
		_points.push_back(_pendingBullet);
		_fillBullet = false;
	}
	this->m_externalForce = ZERO_VEC;
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

	_isPressed = true;

	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	_isPressed = false;

	_isDragging = false;

	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouseDown(int x, int y)
{
	if (_enableShoot) {
		shootPointMass(x, y);
		_isDragging = false;
	}
	else {
		selectDragPoint(x, y, _dragType, _dragIndex);
		if (_dragIndex != -1) {
			_isDragging = true;

		}
		else {
			_isDragging = false;
		}
	}
}

void RigidBodySystemSimulator::onMouseDouble(int x, int y)
{
	if (_enableShoot)
		return;

	selectDragPoint(x, y, _dragType, _dragIndex);
	if (_dragIndex != -1) {
		if (_dragType == 0) {
			PointMass& p = _points[_dragIndex];
			p._isFixed = !p._isFixed;
			if (p._isFixed) {
				p._velocity = ZERO_VEC;
				p._acceleration = ZERO_VEC;
				p._force = ZERO_VEC;
			}
		}
		else if (_dragType == 1) {
			RigidBody& b = _rigidbodies[_dragIndex];
			b._isFixed = !b._isFixed;
			if (b._isFixed) {
				b._velocity = ZERO_VEC;
				b._angularMomentum = ZERO_VEC;
				b._angularVelocity = ZERO_VEC;
				b._force = ZERO_VEC;
				b._torque = ZERO_VEC;
			}
		}
	}
}

void RigidBodySystemSimulator::selectDragPoint(int x, int y, int& outType, int& outIndex)
{
	// find closest point mass
	float mx = (float)x / DUC->g_windowSize[0] * 2 - 1;
	float my = 1 - (float)y / DUC->g_windowSize[1] * 2;

	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();

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
	normalize(rayDir);
	rayDir = ViewInv.transformVectorNormal(rayDir);

	const float dragTolerance = 2e-3;
	const float overlapTolerance = 1e-3;
	float nearestZ = 1;
	float mindd = 1;

	outIndex = -1;
	outType = -1;

	for (int i = 0; i < POINT_SIZE; i++) {
		XMVECTOR clip_pos = XMVector4Transform(_points[i]._position.toDirectXVector(), viewProj);
		XMVECTOR w_vector = XMVectorSplatW(clip_pos);
		XMVECTOR ndc = XMVectorDivide(clip_pos, w_vector);

		float px = DirectX::XMVectorGetX(ndc);
		float py = DirectX::XMVectorGetY(ndc);
		float pz = DirectX::XMVectorGetZ(ndc); // depth

		float dd = (mx - px) * (mx - px) + (my - py) * (my - py);
		if (dd < dragTolerance) {
			if (mindd > overlapTolerance) {
				if (mindd > dd) {
					mindd = dd;
					nearestZ = pz;
					outIndex = i;
					outType = 0;
					_dragOrigin = _points[i]._position;
				}
			}
			else if (nearestZ > pz && dd < overlapTolerance) {
				nearestZ = pz;
				outIndex = i;
				outType = 0;
				_dragOrigin = _points[i]._position;
			}
		}
	}

	Vec3 intersection;
	for (int i = 0; i < BODY_SIZE; i++) {
		RigidBody& body = _rigidbodies[i];
		if (body.rayIntersection(rayOrigin, rayDir, intersection)) {
			XMVECTOR clip_pos = XMVector4Transform(intersection.toDirectXVector(), viewProj);
			XMVECTOR w_vector = XMVectorSplatW(clip_pos);
			XMVECTOR ndc = XMVectorDivide(clip_pos, w_vector);
			float px = DirectX::XMVectorGetX(ndc);
			float py = DirectX::XMVectorGetY(ndc);
			float pz = DirectX::XMVectorGetZ(ndc); // depth

			float dd = (mx - px) * (mx - px) + (my - py) * (my - py);
			if (dd < dragTolerance) {
				if (mindd > overlapTolerance) {
					if (mindd > dd) {
						mindd = dd;
						nearestZ = pz;
						outIndex = i;
						outType = 1;
						_dragOrigin = intersection;
					}
				}
				else if (nearestZ > pz && dd < overlapTolerance) {
					nearestZ = pz;
					outIndex = i;
					outType = 1;
					_dragOrigin = intersection;
				}
			}
		}
	}

	_dragMousePoint = _dragOrigin;
	if (outType == 0) {
		_dragObjectPoint = ZERO_VEC;
	}
	else if (outType == 1) {
		_dragObjectPoint = 
			_rigidbodies[outIndex]._invTransformation.transformVector(_dragOrigin);
	}

}

void RigidBodySystemSimulator::shootPointMass(int x, int y)
{
	if (_fillBullet)
		return;

	// find closest point mass
	float mx = (float)x / DUC->g_windowSize[0] * 2 - 1;
	float my = 1 - (float)y / DUC->g_windowSize[1] * 2;

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
	normalize(rayDir);
	rayDir = ViewInv.transformVectorNormal(rayDir);

	PointMass bullet(0.5, rayOrigin, rayDir * 5, 0.04, false, true);
	_pendingBullet = bullet;
	_fillBullet = true;
}

int RigidBodySystemSimulator::addMassPoint(float mass, Vec3 position, Vec3 Velocity, bool isFixed, float radius)
{
	int point_idx = -1;

	_points.push_back(PointMass(mass, position, Velocity, radius, isFixed));

	point_idx = _points.size() - 1;

	return point_idx;
}

void RigidBodySystemSimulator::addSpring(float damping, float stiffness, int masspoint1, int masspoint2)
{
	float initialLength = norm(_points[masspoint1]._position - _points[masspoint2]._position);
	addSpring(damping, stiffness, masspoint1, masspoint2, initialLength);
}
void RigidBodySystemSimulator::addSpring(float damping, float stiffness, int masspoint1, int masspoint2, float initialLength)
{
	this->_springs.push_back(Spring(masspoint1, masspoint2, stiffness, damping, initialLength));
}

void RigidBodySystemSimulator::loadSimpleSetup()
{
	reset();

	_enableCollision = false;
	_enableWallCollision = false;
	_enableFloorCollision = false;
	_enableGraviy = false;

	Vec3 p0(0, 0, 0);
	Vec3 size(1, 0.6, 0.5);
	float m0 = 2;
	float rz = 90;

	_externalSpringForce = 1;
	_linearDamping = 0;
	_angularDamping = 0;
	_gravity = ZERO_VEC;

	addRigidBody(p0, size, m0);
	setOrientationOf(0, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.5f));
	applyForceOnBody(0, Vec3(0.3f, 0.5f, 0.25f), Vec3(1.0f, 1.0f, 0.0f));
}

void RigidBodySystemSimulator::loadTwoBodySetup()
{
	reset();

	_enableCollision = true;
	_enableWallCollision = false;
	_enableFloorCollision = false;
	_enableGraviy = false;

	_externalSpringForce = 1;
	_linearDamping = 3.5;
	_angularDamping = 0.05;
	_gravity = ZERO_VEC;

	addRigidBody(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), 100.0f, false);
	addRigidBody(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), 100.0);
	setOrientationOf(1, Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f));
	setVelocityOf(1, Vec3(0.0f, -0.1f, 0.05f));
}

void RigidBodySystemSimulator::loadComplexSetup()
{
	reset();

	_enableCollision = true;
	_enableWallCollision = true;
	_enableFloorCollision = true;
	_enableGraviy = true;

	_linearDamping = 0.18;
	_angularDamping = 0.05;

	_externalSpringForce = 3.5;

	Vec3 gravity(0, -0.065, 0);

	_gravity = gravity;

	// at least 10 points and springs
	// ground or wall collisions
	// rope 0
	//this->createRope(1, springDamping, 15, Vec3(0.2, 0.4, 0), Vec3(0.2, 0.025, 0), 3);
	// rope 1
	//this->createRope(1, springDamping, 40, Vec3(-0.2, 0.4, 0), Vec3(-0.2, 0.025, 0), 3);
	// box 0
	this->createBox(1, 0.5, 30, Vec3(0, 0.25, 0), 0.08);
	addRigidBody(Vec3(0, -1.0, 0), Vec3(0.3, 0.1, 0.3), 5, false);
	addRigidBody(Vec3(0, 0.1, 0), Vec3(0.24, 0.05, 0.24), 3, false);
	addRigidBody(Vec3(0, 0.0, 0), Vec3(0.25, 0.11, 0.25), 4, false);
	addRigidBody(Vec3(0, 0.7, 0), Vec3(0.22, 0.1, 0.22), 3, false);
	addRigidBody(Vec3(0, 1.0, 0), Vec3(0.22, 0.22, 0.22), 5, false);
	// cloth 0
	this->createCloth(0.05, 0.85, 15, Vec3(-0.2, 0.4, 0.1), Vec3(0.2, 0.1, 0.1), 3, 4);
}

void RigidBodySystemSimulator::createRope(float mass, float damping, float stiffness, const Vec3& start, const Vec3& end, int samples)
{
	int istart = this->addMassPoint(mass, start, ZERO_VEC, true);

	Vec3 ropeVec = end - start;
	float length = normalize(ropeVec);
	float unitLength = length / (samples + 1);
	for (int n = 1; n <= samples; n++) {
		int in = istart + n;
		this->addMassPoint(mass, start + unitLength * ropeVec * n, ZERO_VEC, false);
		this->addSpring(damping, stiffness, in - 1, in, unitLength);
	}

	int iend = this->addMassPoint(mass, end, ZERO_VEC, false);
	this->addSpring(damping, stiffness, iend - 1, iend, unitLength);

}

void RigidBodySystemSimulator::createCloth(float mass, float damping, float stiffness, const Vec3& start, const Vec3& end, int samples0, int samples1)
{
	const int CLOTH_SIZE = 6;
	const float UNIT_LENGTH = 0.081;
	vector<vector<int>> p;

	for (int x = 0; x < CLOTH_SIZE; x++) {
		vector<int> row;
		for (int z = 0; z < CLOTH_SIZE; z++) {
			bool isFixed = false;
			Vec3 v = Vec3(0, 0, 0);
			float y = 0.5;
			if(x == 0 || z == 0 || x == CLOTH_SIZE - 1 || z == CLOTH_SIZE - 1)
				isFixed = true;

			row.push_back(addMassPoint(mass, Vec3(x * UNIT_LENGTH - (CLOTH_SIZE / 2) * UNIT_LENGTH, y, z * UNIT_LENGTH - (CLOTH_SIZE / 2) * UNIT_LENGTH), v, isFixed));
		}
		p.push_back(row);
	}

	// Structural
	for (int x = 0; x < CLOTH_SIZE; x++) {
		for (int z = 0; z < CLOTH_SIZE; z++) {
			//STRUCTURAL
			if (z + 1 < CLOTH_SIZE) 
				addSpring(damping, stiffness, p[x][z], p[x][z + 1], UNIT_LENGTH);
			if (x + 1 < CLOTH_SIZE) 
				addSpring(damping, stiffness, p[x][z], p[x + 1][z], UNIT_LENGTH);

			//SHEAR	
			if (x + 1 < CLOTH_SIZE && z + 1 < CLOTH_SIZE) 
				addSpring(damping, stiffness, p[x][z], p[x + 1][z + 1], sqrt(pow(UNIT_LENGTH, 2) + pow(UNIT_LENGTH, 2)));
			if (x - 1 >= 0 && z + 1 < CLOTH_SIZE) 
				addSpring(damping, stiffness, p[x][z], p[x - 1][z + 1], sqrt(pow(UNIT_LENGTH, 2) + pow(UNIT_LENGTH, 2)));

			//FLEXION
			if (x + 2 < CLOTH_SIZE) 
				addSpring(damping, stiffness, p[x][z], p[x + 2][z], UNIT_LENGTH * 2.0);
			if (z + 2 < CLOTH_SIZE) 
				addSpring(damping, stiffness, p[x][z], p[x][z + 2], UNIT_LENGTH * 2.0);

		}
	}
}

void RigidBodySystemSimulator::createBox(float mass, float damping, float stiffness, const Vec3& center, const float size)
{
	int istart = _points.size() - 1;

	this->addMassPoint(mass, center + Vec3(size, -size, size), ZERO_VEC, false);
	this->addMassPoint(mass, center + Vec3(size, size, size), ZERO_VEC, false);
	this->addMassPoint(mass, center + Vec3(-size, size, size), ZERO_VEC, false);
	this->addMassPoint(mass, center + Vec3(-size, -size, size), ZERO_VEC, false);

	this->addMassPoint(mass, center + Vec3(size, -size, -size), ZERO_VEC, false);
	this->addMassPoint(mass, center + Vec3(size, size, -size), ZERO_VEC, false);
	this->addMassPoint(mass, center + Vec3(-size, size, -size), ZERO_VEC, false);
	this->addMassPoint(mass, center + Vec3(-size, -size, -size), ZERO_VEC, false);

	// inner
	this->addSpring(damping, stiffness, istart + 7, istart + 1);
	this->addSpring(damping, stiffness, istart + 8, istart + 2);
	this->addSpring(damping, stiffness, istart + 5, istart + 3);
	this->addSpring(damping, stiffness, istart + 4, istart + 6);
	// outter
	this->addSpring(damping, stiffness, istart + 2, istart + 1);
	this->addSpring(damping, stiffness, istart + 2, istart + 3);
	this->addSpring(damping, stiffness, istart + 3, istart + 4);
	this->addSpring(damping, stiffness, istart + 1, istart + 4);
	this->addSpring(damping, stiffness, istart + 5, istart + 1);
	this->addSpring(damping, stiffness, istart + 2, istart + 6);
	this->addSpring(damping, stiffness, istart + 3, istart + 7);
	this->addSpring(damping, stiffness, istart + 4, istart + 8);
	this->addSpring(damping, stiffness, istart + 5, istart + 6);
	this->addSpring(damping, stiffness, istart + 7, istart + 6);
	this->addSpring(damping, stiffness, istart + 7, istart + 8);
	this->addSpring(damping, stiffness, istart + 5, istart + 8);
	// cross
	this->addSpring(damping, stiffness, istart + 1, istart + 3);
	this->addSpring(damping, stiffness, istart + 6, istart + 1);
	this->addSpring(damping, stiffness, istart + 1, istart + 8);
	this->addSpring(damping, stiffness, istart + 2, istart + 4);
	this->addSpring(damping, stiffness, istart + 5, istart + 2);
	this->addSpring(damping, stiffness, istart + 2, istart + 7);
	this->addSpring(damping, stiffness, istart + 3, istart + 6);
	this->addSpring(damping, stiffness, istart + 3, istart + 8);
	this->addSpring(damping, stiffness, istart + 4, istart + 5);
	this->addSpring(damping, stiffness, istart + 4, istart + 7);
	this->addSpring(damping, stiffness, istart + 7, istart + 5);
	this->addSpring(damping, stiffness, istart + 6, istart + 8);
}

void RigidBodySystemSimulator::collisionResolve(
	const float& deltaTime)
{
	collisionRigidBodies(deltaTime);
	if (_enableFloorCollision) {
		collisionPLane(deltaTime, Vec3(0, 1, 0), -0.5, 10);
	}
	if (_enableWallCollision) {
		collisionPLane(deltaTime, Vec3(1, 0, 0), -0.5, 0.1);
		collisionPLane(deltaTime, Vec3(-1, 0, 0), -0.5, 0.1);
		collisionPLane(deltaTime, Vec3(0, 0, 1), -0.5, 0.1);
		collisionPLane(deltaTime, Vec3(0, 0, -1), -0.5, 0.1);
	}
}
void RigidBodySystemSimulator::collisionPLane(
	const float& deltaTime,
	const Vec3& n, const float offset, const float friction)
{
	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();
	const float epsilon = 1e-9;

	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = _points[pid];
		if (p._isBullet)
			break;
		if (!p._isFixed) {
			float d = dot(p._position, n);
			Vec3 vel = p._velocity;
			float velnorm = normalize(vel);
			float velDot = dot(vel, n);
			if (d < offset && velDot < 0) {
				Vec3 parallelVel = p._velocity - n * velnorm * velDot;
				Vec3 parallelDir = parallelVel;
				double parallelnorm = normalize(parallelDir);
				float distance = (offset - d + epsilon) / velDot;
				p._position = p._position + distance * vel;
				p._velocity = parallelVel;
				const float bounciness = 0.55;
				float J = -p._mass * (bounciness) * velnorm * velDot;
				p._force = (J / (deltaTime + epsilon)) * n 
					+ min(max(friction * dot(p._force, n), -p._mass * parallelnorm / (deltaTime + epsilon)), 0.0) * parallelDir;
			}
		}
	}
	std::vector<Contact> hits;
	for (int bid = 0; bid < BODY_SIZE; bid++) {
		RigidBody& body = _rigidbodies[bid];
		if (!body._isSleep && !body._isFixed && body.planeContact(n, offset, hits)) {
			Vec3 accPos(0, 0, 0);
			Vec3 accVel(0, 0, 0);
			Vec3 accAng(0, 0, 0);

			float weight = 1.0 / hits.size();

			float iM0 = body._invMass;
			Mat3 iI0 = body._worldInvInertia;
			for (int cid = 0; cid < hits.size(); cid++) {
				Contact& hit = hits[cid];
				const Vec3 x0 = (hit.contactPoint - body._worldCom);
				const Vec3& n = hit.contactNormal;
				Vec3 vrel = body._velocity + cross(body._angularVelocity, x0);
				float vdot = dot(vrel, n);

				Vec3 parallelVel = vrel - n * vdot;
				Vec3 parallelDir = parallelVel;
				double parallelnorm = normalize(parallelDir);

				Vec3 frictionForce = weight * min(max(friction * dot(body._force, n),
					-body._mass * parallelnorm / (deltaTime + epsilon)), 0.0) * parallelDir;
				body.addExternalForce(frictionForce, hit.contactPoint);

				if (vdot < 0) {
					const float c = 0;
					float J = (-(1 + c) * vdot) /
						(iM0 + dot(
							cross(iI0.dot(cross(x0, n)), x0), n));
					const Vec3 Jn = J * n;
					accVel += weight * Jn * iM0;
					accAng += weight * cross(x0, Jn);
				}
				accPos += weight * n * hit.contactDistance;
			}

			if ((normNoSqrt(accVel) + normNoSqrt(accAng)) > 1e-7) {
				body.wake();
				body._position += accPos;
				body._velocity += accVel;
				body._angularMomentum += accAng;
			}
		}
	}
}

void RigidBodySystemSimulator::collisionRigidBodies(const float& deltaTime)
{
	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();
	const float epsilon = 1e-9;
	// points to rigidbodies
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = _points[pid];
		for (int bid = 0; bid < BODY_SIZE; bid++) {
			RigidBody& body = _rigidbodies[bid];
			if ((p._isFixed) && (body._isSleep || body._isFixed))
				continue;
			// fast check with simple test first
			if (body.isOverlapCoarse(p._position, p._radius)) {
				Contact hit;
				if (body.pointContact(p, hit)) {
					float iM0 = 0;
					float iM1 = 0;
					Mat3 iI0;
					Vec3 v0(0, 0, 0);
					Vec3 v1(0, 0, 0);
					const Vec3 x0 = (hit.contactPoint - body._worldCom);
					const Vec3& n = hit.contactNormal;
					if (!body._isFixed) {
						iM0 = body._invMass;
						iI0 = body._worldInvInertia;
						v0 = body._velocity + cross(body._angularVelocity, x0);
					}
					if (!p._isFixed) {
						iM1 = p._invMass;
						v1 = p._velocity;
					}
					Vec3 vrel = v0 - v1;
					float vdot = dot(vrel, n);
					if (vdot < 0 || body._isFixed) {
						const float c = 0;
						float J = (-(1 + c) * vdot) /
							(iM0 + iM1 + dot(
								cross(iI0.dot(cross(x0, n)), x0), n));
						const Vec3 Jn = J * n;
						if (!body._isFixed) {
							//Vec3 posDiff = (iM0 / (iM0 + iM1)) * hit.contactDistance * hit.contactNormal;
							Vec3 posDiff = 0.5 * hit.contactDistance * hit.contactNormal;
							Vec3 velDiff = Jn * iM0;
							Vec3 angDiff = cross(x0, Jn);
							if ((normNoSqrt(velDiff) + normNoSqrt(angDiff) + normNoSqrt(posDiff)) > 1e-6) {
								body.wake();
								body._position += posDiff;
								body._velocity += velDiff;
								body._angularMomentum += angDiff;
							}
							//body._force = ZERO_VEC;
							//body._torque = ZERO_VEC;
						}
						if (!p._isFixed) {
							//p._position += (iM1 / (iM0 + iM1)) * -hit.contactDistance * hit.contactNormal;
							p._position += 0.5 * -hit.contactDistance * hit.contactNormal;
							p._velocity += -Jn * iM1;
							p._force = ZERO_VEC;
						}
					}
				}
			}
		}
	}
	// rigidbodies to rigidbodies
	for (int bid = 0; bid < BODY_SIZE; bid++) {
		RigidBody& b0 = _rigidbodies[bid];
		for (int cid = bid+1; cid < BODY_SIZE; cid++) {
			RigidBody& b1 = _rigidbodies[cid];
			if ((b0._isSleep || b0._isFixed) && (b1._isSleep || b1._isFixed))
				continue;
			// fast check with simple test first
			if (b0.isOverlapCoarse(b1._coarseCenter, b1._coarseRadius)) 
			{
				// check with the provided method
				CollisionInfo hit = checkCollisionSAT(
					b0._renderTransformation, b1._renderTransformation);
				// rigidbody collides
				if (hit.isValid) {
					// handle collision
					b0.wake();
					b1.wake();
					// might be better to update the sleep states at once later 
					// since the property is used in the previous if/else check
					float iM0 = 0;
					float iM1 = 0;
					Mat3 iI0;
					Mat3 iI1;
					Vec3 v0(0, 0, 0);
					Vec3 v1(0, 0, 0);
					const Vec3 x0 = (hit.collisionPointWorld - b0._worldCom);
					const Vec3 x1 = (hit.collisionPointWorld - b1._worldCom);
					const Vec3& n = hit.normalWorld;
					if (!b0._isFixed) {
						iM0 = b0._invMass;
						iI0 = b0._worldInvInertia;
						v0 = b0._velocity + cross(b0._angularVelocity, x0);
					}
					if (!b1._isFixed) {
						iM1 = b1._invMass;
						iI1 = b1._worldInvInertia;
						v1 = b1._velocity + cross(b1._angularVelocity, x1);
					}
					Vec3 vrel = v0 - v1;
					float vdot = dot(vrel, n);
					if (vdot < 0) {
						const float c = 0;
						float J = (-(1 + c) * vdot) /
							(iM0 + iM1 + dot(
								cross(iI0.dot(cross(x0, n)), x0) +
								cross(iI1.dot(cross(x1, n)), x1), n));
						const Vec3 Jn = J * n;
						if (!b0._isFixed) {
							b0._position += 0.5 * hit.depth * hit.normalWorld;
							b0._velocity += Jn * iM0;
							b0._angularMomentum += cross(x0, Jn);
						}
						if (!b1._isFixed) {
							b1._position -= 0.5 * hit.depth * hit.normalWorld;
							b1._velocity -= Jn * iM1;
							b1._angularMomentum -= cross(x1, Jn);
						}
					}
				}
			}
		}
	}
}

void RigidBodySystemSimulator::computeSpringForce()
{
	const int SPRING_SIZE = _springs.size();

	// force computation
	// calculate internal forces (spring forces)
	for (int sid = 0; sid < SPRING_SIZE; sid++) {
		const Spring& s = _springs[sid];

		int pid0 = s._pointID0;
		int pid1 = s._pointID1;

		PointMass& p0 = _points[pid0];
		PointMass& p1 = _points[pid1];

		// hook's law
		Vec3 v0 = p0._position - p1._position;
		float length = normalize(v0);
		// forces
		Vec3 f0 = -(s._stiffness) * (length - s._restLength) * v0
			- s._damping * dot(p0._velocity - p1._velocity, v0) * v0;
		p0._force += f0;
		p1._force += -f0;
	}
}

void RigidBodySystemSimulator::dampingForce()
{
	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();
	// damping forces
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = _points[pid];
		if (p._isBullet)
			break;
		p._force += -_linearDamping * p._velocity;
	}
	for (int rid = 0; rid < BODY_SIZE; rid++) {
		RigidBody& body = _rigidbodies[rid];
		body._force += -_linearDamping * body._velocity;
		body._torque += -_angularDamping * body._angularVelocity;
	}
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	return _rigidbodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	return _rigidbodies[i]._position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	return _rigidbodies[i]._velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	return _rigidbodies[i]._angularVelocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	_rigidbodies[i].addExternalForce(force, loc);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass, bool fixed)
{
	_rigidbodies.push_back(RigidBody(mass, size, position, fixed));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	_rigidbodies[i]._orientation = orientation;
	_rigidbodies[i].updateTransofmration();
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	_rigidbodies[i]._velocity = velocity;
}

void RigidBodySystemSimulator::applyExternalForce()
{
	if (_isDragging && _dragIndex != -1) {
		if (_dragType == 0 && !_points[_dragIndex]._isFixed) {
			const float sitffness = _externalSpringForce;
			// hook's law
			Vec3 v0 = _points[_dragIndex]._position - _dragMousePoint;
			float length = normalize(v0);
			// forces
			_points[_dragIndex]._force += -sitffness * length * v0;
		}
		else if (_dragType == 1 && !_rigidbodies[_dragIndex]._isFixed) {
			RigidBody& body = _rigidbodies[_dragIndex];
			const float sitffness = _externalSpringForce;
			// hook's law
			Vec3 dragWorldPoint = body._transformation.transformVector(_dragObjectPoint);
			Vec3 v0 = dragWorldPoint - _dragMousePoint;
			float length = normalize(v0);
			// forces
			Vec3 force = -sitffness * length * v0;
			if (normNoSqrt(force) > 1e-5) {
				_rigidbodies[_dragIndex].wake();
				_rigidbodies[_dragIndex].addExternalForce(force, dragWorldPoint);
			}
		}
	}
}

void RigidBodySystemSimulator::updatePosition(const float& timeStep)
{
	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();

	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = _points[pid];
		if (!p._isFixed)
		{
			p._lastPosition = p._position;
			p._position = p._position + p._velocity * timeStep;
		}
	}
	for (int rid = 0; rid < BODY_SIZE; rid++) {
		RigidBody& body = _rigidbodies[rid];
		if (!body._isFixed && !body._isSleep)
		{
			body._lastPosition = body._position;
			body._position = body._position + body._velocity * timeStep;
			const Vec3& vw = body._angularVelocity;
			Quat w(vw.x, vw.y, vw.z, 0);
			//[TODO] WTF ?? This order below is incorrect in my opinion, 
			// but it will pass the unit test ???
			//body._orientation += (timeStep * 0.5)  * body._orientation * w;
			//
			body._orientation += (timeStep * 0.5) * w * body._orientation;
			float quatnorm = body._orientation.norm();
			if (quatnorm == 0)
			{
				body._orientation = Quat(0, 0, 0, 1);
			}
			else {
				body._orientation /= quatnorm;
			}
			body.updateTransofmration();
		}
	}
}

void RigidBodySystemSimulator::updateVelocity(
	const float& timeStep)
{
	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();

	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = _points[pid];
		if (!p._isFixed)
		{
			p._acceleration = p._force / (p._mass);
			p._velocity = p._velocity + p._acceleration * timeStep;
		}
		else {
			p._acceleration = ZERO_VEC;
			p._velocity = ZERO_VEC;
		}
	}
	for (int rid = 0; rid < BODY_SIZE; rid++) {
		RigidBody& body = _rigidbodies[rid];
		if (!body._isFixed)
		{
			if (!body._isSleep) {
				body._velocity += timeStep * (body._force / body._mass);
				body._angularMomentum += timeStep * body._torque;
				/*Vec3 localL = 
					body._invTransformation.transformVectorNormal(body._angularMomentum);
				Vec3 localW = body._invInertia.dot(localL);
				body._angularVelocity = body._transformation.transformVectorNormal(localW);*/
				body._angularVelocity = body._worldInvInertia.dot(body._angularMomentum);
				body.checkSleep();
			}
		}
		else {
			body._velocity = ZERO_VEC;
			body._angularVelocity = ZERO_VEC;
		}
	}
}

void RigidBodySystemSimulator::integrateExplicitEuler(const float& timeStep)
{
	if (_enableGraviy) {
		applyGravity(_gravity);
	}
	computeSpringForce();
	dampingForce();
	if (_enableCollision) {
		collisionResolve(timeStep);
	}
	applyExternalForce();
	updatePosition(timeStep);
	updateVelocity(timeStep);
	clearForce();
}

void RigidBodySystemSimulator::integrateLeapfrog(const float& timeStep)
{
	if (_enableGraviy) {
		applyGravity(_gravity);
	}
	computeSpringForce();
	dampingForce();
	applyExternalForce();
	updateVelocity(timeStep);
	updatePosition(timeStep);
	clearForce();
	if (_enableCollision) {
		collisionResolve(timeStep);
	}
}

void RigidBodySystemSimulator::clearForce()
{
	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();

	// clear forces from last step
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		_points[pid]._force = ZERO_VEC;
	}
	for (int rid = 0; rid < BODY_SIZE; rid++) {
		_rigidbodies[rid]._force = ZERO_VEC;
		_rigidbodies[rid]._torque = ZERO_VEC;
	}
}

void RigidBodySystemSimulator::applyGravity(const Vec3& g)
{
	const int POINT_SIZE = _points.size();
	const int BODY_SIZE = _rigidbodies.size();
	// external forces
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		if (_points[pid]._isBullet)
			break;
		_points[pid]._force += _points[pid]._mass * g;
	}
	for (int rid = 0; rid < BODY_SIZE; rid++) {
		_rigidbodies[rid]._force += _rigidbodies[rid]._mass * g;
	}
}

RigidBodySystemSimulator::Bullet::Bullet(int index_, float lifeTime_)
{
	this->index = index_;
	this->lifeTime = lifeTime_;
}
