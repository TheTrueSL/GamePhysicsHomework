#include "MassSpringSystemSimulator.h"

#include<random>
#include<time.h>

const Vec3 ZERO_VEC(0, 0, 0);

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	DUC         = nullptr;

	m_fMass       = 1;
	m_fStiffness  = 1;
	m_fPointDamping = 0;
	m_fSpringDamping = 0;
	m_iIntegrator = 0;

	_enableCollision = false;
	_enableFakeImpact = true;
	_enableGraviy = false;
	_enableExternalSpringForce = false;

	_printSteps = -1;

	reset();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	// custom case name, split with comma","
	// notifyCaseChanged(int index) will be called when case is changed by user.
	return "Demo1(simple 1 step), Demo2&3(simple), Demo4&5(complex)"; 
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator Type", 
		"Euler, Leapfrog, Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &this->m_iIntegrator, "");

	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &this->_enableGraviy, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_x", TW_TYPE_DOUBLE, &this->_gravity.x, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_y", TW_TYPE_DOUBLE, &this->_gravity.y, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity_z", TW_TYPE_DOUBLE, &this->_gravity.z, "");
	TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_BOOLCPP, &this->_enableCollision, "");
	TwAddVarRW(DUC->g_pTweakBar, "FakeImpact", TW_TYPE_BOOLCPP, &this->_enableFakeImpact, "");
	TwAddVarRW(DUC->g_pTweakBar, "ExternalForce", TW_TYPE_FLOAT, &this->_externalSpringForce, "");
	
}

void MassSpringSystemSimulator::reset()
{
	_points.clear();
	_tempPoints.clear();
	_springs.clear();

	_isPressed = false;
	_pressedTimer = clock();
	// UI Attributes
	_externalSpringForce = 0;

	m_externalForce = Vec3();
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::vector<PointMass>& points = _points;

	const int POINT_SIZE = points.size();
	const int SPRING_SIZE = _springs.size();

	const float pointScale = 0.01f;
	const Vec3 lineColor0 = Vec3(1., 1., 1.);
	const Vec3 lineColor1 = Vec3(1., 0., 0.);
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = points[pid];
		if (p._isFixed) {
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, Vec3(1, 0, 0));
		}
		else {
			DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 100, Vec3(1, 1, 1));
		}
		this->DUC->drawSphere(p._position, pointScale);
	}
	DUC->setUpLighting(Vec3(), Vec3(1, 1, 1), 0, Vec3(1, 1, 1));

	this->DUC->beginLine();
	for (int sid = 0; sid < SPRING_SIZE; sid++) {
		Spring& s = _springs[sid];
		int pid0 = s._pointID0;
		int pid1 = s._pointID1; 
		{
			PointMass& p0 = points[pid0];
			PointMass& p1 = points[pid1];
			this->DUC->drawLine(p0._position, lineColor0, p1._position, lineColor1);
		}
	}

	if (_enableExternalSpringForce) {
		this->DUC->drawLine(_dragPoint, lineColor1, points[_dragPointMassIndex]._position, lineColor1);
	}

	this->DUC->endLine();

	/*if (_enableExternalSpringForce) {
		this->DUC->drawSphere(_dragPoint, 0.1);
	}*/
	
	//this->DUC->DrawTriangleUsingShaders();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	std::cout << "Case Changed : " << testCase << std::endl;
	switch (testCase)
	{
	case 0:
		_printSteps = 1;
		_enableCollision = false;
		_enableGraviy = false;
		loadSimpleSetup();
		break;
	case 1:
		_printSteps = -1;
		_enableCollision = false;
		_enableGraviy = false;
		loadSimpleSetup();
		break;
	case 2:
		_printSteps = -1;
		_enableCollision = true;
		_enableGraviy = true;
		loadComplexSetup();
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
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
		_dragPoint = ViewInv.transformVector(viewDragPoint);

		if (_dragPointMassIndex != -1 && _points[_dragPointMassIndex]._isFixed) {
			_points[_dragPointMassIndex]._position = _dragPoint;
		}
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (_printSteps == 0)
		return;

	if (m_iIntegrator == EULER) {
		integrateExplicitEuler(timeStep, 
			_points, _springs, this->m_externalForce,
			_points);
	}
	else if (m_iIntegrator == MIDPOINT) {
		integrateMidpoint(timeStep, 
			_points, _springs, this->m_externalForce,
			_points);
	}
	else if (m_iIntegrator == LEAPFROG) {
		integrateLeapfrog(timeStep, 
			_points, _springs, this->m_externalForce,
			_points);
	}
	else {
		integrateExplicitEuler(timeStep, 
			_points, _springs, this->m_externalForce,
			_points);
	}

	if (_printSteps > 0) {
		std::vector<PointMass>& points = _points;
		for (int i = 0; i < points.size(); i++) {
			std::cout << "p " << i << ": " << points[i]._position << std::endl;
			std::cout << "v " << i << ": " << points[i]._velocity << std::endl;
		}
		_printSteps -= 1;
	}
	this->m_externalForce = ZERO_VEC;
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	if (!_isPressed) {

		time_t hoverTime = _pressedTimer;
		_pressedTimer = clock();
		hoverTime = _pressedTimer - hoverTime;

		if (hoverTime < 500) { // 500ms
			onMouseDouble(x, y);
			_pressedTimer = _pressedTimer - 500;
		}
		else {
			onMouseDown(x, y);
		}
	}

	_isPressed = true;

	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	_isPressed = false;

	_enableExternalSpringForce = false;

	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouseDown(int x, int y)
{
	selectMassPoint(x, y, _dragPointMassIndex);
	if (_dragPointMassIndex != -1) {
		PointMass& p = _points[_dragPointMassIndex];
		_enableExternalSpringForce = true;
		_dragOrigin = p._position;
		_dragPoint = _dragOrigin;
	}
	else {
		_enableExternalSpringForce = false;
	}
}

void MassSpringSystemSimulator::onMouseDouble(int x, int y)
{
	selectMassPoint(x, y, _dragPointMassIndex);
	if (_dragPointMassIndex != -1) {
		PointMass& p = _points[_dragPointMassIndex];
		p._isFixed = !p._isFixed;
		if (p._isFixed) {
			p._velocity = ZERO_VEC;
			p._acceleration = ZERO_VEC;
			p._force = ZERO_VEC;
		}
	}
}

void MassSpringSystemSimulator::selectMassPoint(int x, int y, int& outIndex)
{
	// find closest point mass
	float mx = (float)x / DUC->g_windowSize[0] * 2 - 1;
	float my = 1 - (float)y / DUC->g_windowSize[1] * 2;

	const int POINT_SIZE = _points.size();
	XMMATRIX viewProj = DUC->g_camera.GetViewMatrix() *
		DUC->g_camera.GetProjMatrix();

	const float dragTolerance = 1e-2;
	const float overlapTolerance = 1e-3;
	float nearestZ = 1;
	float mindd = 1;

	outIndex = -1;

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
				}
			}
			else {
				if (nearestZ > pz && dd < overlapTolerance) {
					nearestZ = pz;
					outIndex = i;
				}
			}
		}
	}


}


void MassSpringSystemSimulator::setMass(float mass)
{
	this->m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setGravity(const Vec3& gravity)
{
	this->_gravity = gravity;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	this->m_fPointDamping = damping;
}

void MassSpringSystemSimulator::setSpringDampingFactor(float damping)
{
	this->m_fSpringDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int point_idx = -1;

	float mass = this->m_fMass;
	float damping = this->m_fPointDamping;

	_points.push_back(PointMass(mass, damping, position, Velocity, isFixed));
	_tempPoints.push_back(PointMass(mass, damping, position, Velocity, isFixed));

	point_idx = _points.size() - 1;

	return point_idx;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2)
{
	float stiffness = this->m_fStiffness;
	float damping = this->m_fSpringDamping;

	float initialLength = norm(_points[masspoint1]._position - _points[masspoint2]._position);
	this->_springs.push_back(Spring(masspoint1, masspoint2, stiffness, damping, initialLength));

}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	float stiffness = this->m_fStiffness;
	float damping = this->m_fSpringDamping;

	this->_springs.push_back(Spring(masspoint1, masspoint2, stiffness, damping, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return _points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return this->_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return _points[index]._position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return _points[index]._velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	this->m_externalForce += force;
}

void MassSpringSystemSimulator::loadSimpleSetup()
{
	reset();
	Vec3 p0(0, 0, 0);
	Vec3 p1(0, 2, 0);
	Vec3 v0(-1, 0, 0);
	Vec3 v1(1, 0, 0);
	float m0 = 10;
	float m1 = 10;

	float l0 = 1;
	float stiffness0 = 40;

	float damping = 0;
	Vec3 gravity(0, 0, 0);

	this->setDampingFactor(damping);
	this->setGravity(gravity);

	this->setMass(m0);
	this->addMassPoint(p0, v0, false);
	
	this->setMass(m1);
	this->addMassPoint(p1, v1, false);

	this->setStiffness(stiffness0);
	this->addSpring(0, 1, l0);

}

void MassSpringSystemSimulator::loadComplexSetup()
{
	reset();

	float damping = 0.15;
	float springDamping = 0.4;

	_externalSpringForce = 3.5;

	Vec3 gravity(0, -0.065, 0);

	this->setDampingFactor(damping);
	this->setSpringDampingFactor(springDamping);
	this->setGravity(gravity);
	// at least 10 points and springs
	// ground or wall collisions
	// rope 0
	this->setMass(1);
	this->setStiffness(15);
	this->createRope(Vec3(0.2, 0.4, 0), Vec3(0.2, 0.025, 0), 3);

	// rope 1
	this->setMass(1);
	this->setStiffness(40);
	this->createRope(Vec3(-0.2, 0.4, 0), Vec3(-0.2, 0.025, 0), 3);

	// cloth 0
	this->setMass(1);
	this->setStiffness(50);
	this->createCloth(Vec3(-0.2, 0.4, 0.1), Vec3(0.2, 0.1, 0.1), 3, 4);

	// box 0
	this->setMass(0.8);
	this->setStiffness(45);
	this->createBox(Vec3(0, 0.3, 0), 0.08);

}

void MassSpringSystemSimulator::createRope(const Vec3& start, const Vec3& end, int samples)
{
	int istart = this->addMassPoint(start, ZERO_VEC, true);

	Vec3 ropeVec = end - start;
	float length = normalize(ropeVec);
	float unitLength = length / (samples + 1);
	for (int n = 1; n <= samples; n++) {
		int in = istart + n;
		this->addMassPoint(start + unitLength * ropeVec * n, ZERO_VEC, false);
		this->addSpring(in - 1, in, unitLength);
	}

	int iend = this->addMassPoint(end, ZERO_VEC, false);
	this->addSpring(iend - 1, iend, unitLength);

}

void MassSpringSystemSimulator::createCloth(const Vec3& start, const Vec3& end, int samples0, int samples1)
{

}

void MassSpringSystemSimulator::createBox(const Vec3& center, const float size)
{
	int istart = this->getNumberOfMassPoints() - 1;

	this->addMassPoint(center + Vec3(size, -size, size), ZERO_VEC, false);
	this->addMassPoint(center + Vec3(size, size, size), ZERO_VEC, false);
	this->addMassPoint(center + Vec3(-size, size, size), ZERO_VEC, false);
	this->addMassPoint(center + Vec3(-size, -size, size), ZERO_VEC, false);

	this->addMassPoint(center + Vec3(size, -size, -size), ZERO_VEC, false);
	this->addMassPoint(center + Vec3(size, size, -size), ZERO_VEC, false);
	this->addMassPoint(center + Vec3(-size, size, -size), ZERO_VEC, false);
	this->addMassPoint(center + Vec3(-size, -size, -size), ZERO_VEC, false);

	// inner
	this->addSpring(istart + 7, istart + 1);
	this->addSpring(istart + 8, istart + 2);
	this->addSpring(istart + 5, istart + 3);
	this->addSpring(istart + 4, istart + 6);
	// outter
	this->addSpring(istart + 2, istart + 1);
	this->addSpring(istart + 2, istart + 3);
	this->addSpring(istart + 3, istart + 4);
	this->addSpring(istart + 1, istart + 4);
	this->addSpring(istart + 5, istart + 1);
	this->addSpring(istart + 2, istart + 6);
	this->addSpring(istart + 3, istart + 7);
	this->addSpring(istart + 4, istart + 8);
	this->addSpring(istart + 5, istart + 6);
	this->addSpring(istart + 7, istart + 6);
	this->addSpring(istart + 7, istart + 8);
	this->addSpring(istart + 5, istart + 8);
	// cross
	this->addSpring(istart + 1, istart + 3);
	this->addSpring(istart + 6, istart + 1);
	this->addSpring(istart + 1, istart + 8);
	this->addSpring(istart + 2, istart + 4);
	this->addSpring(istart + 5, istart + 2);
	this->addSpring(istart + 2, istart + 7);
	this->addSpring(istart + 3, istart + 6);
	this->addSpring(istart + 3, istart + 8);
	this->addSpring(istart + 4, istart + 5);
	this->addSpring(istart + 4, istart + 7);
	this->addSpring(istart + 7, istart + 5);
	this->addSpring(istart + 6, istart + 8);
}

bool MassSpringSystemSimulator::collisionResolve(
	const float& deltaTime,
	const std::vector<PointMass>& points,
	std::vector<PointMass>& outPoints)
{
	bool isCollided = false;
	
	isCollided |= collisionPLane(deltaTime, points, Vec3(0, 1, 0), -0.5, 20, outPoints);
	isCollided |= collisionPLane(deltaTime, points, Vec3(1, 0, 0), -0.5, 1, outPoints);
	isCollided |= collisionPLane(deltaTime, points, Vec3(-1, 0, 0), -0.5, 1, outPoints);
	isCollided |= collisionPLane(deltaTime, points, Vec3(0, 0, 1), -0.5, 1, outPoints);
	isCollided |= collisionPLane(deltaTime, points, Vec3(0, 0, -1), -0.5, 1, outPoints);

	return isCollided;
}
bool MassSpringSystemSimulator::collisionPLane(
	const float& deltaTime,
	const std::vector<PointMass>& points, 
	const Vec3& n, const float offset, const float friction,
	std::vector<PointMass>& outPoints)
{
	const int POINT_SIZE = points.size();
	bool isCollided = false;
	const float epsilon = 1e-9;

	for (int pid = 0; pid < POINT_SIZE; pid++) {
		const PointMass& pIn = points[pid];
		PointMass& pOut = outPoints[pid];
		if (!pIn._isFixed) {
			float d = dot(pIn._position, n);
			Vec3 vel = pIn._velocity;
			float velnorm = normalize(vel);
			float velDot = dot(vel, n);
			if (d < offset && velDot < 0) {
				Vec3 parallelVel = pIn._velocity - n * velnorm * velDot;

				pOut._position = pIn._position + (offset - d + epsilon) * vel / velDot;

				if (_enableFakeImpact) {
					pOut._velocity = parallelVel;
					float bounciness = 0.25;
					float J = -pOut._mass * (bounciness)*velnorm * velDot;
					pOut._force = (J / (deltaTime + epsilon)) * n + (-friction * parallelVel);
				}
				else {
					pOut._velocity = parallelVel;
				}

				isCollided = true;
			}
		}
	}
	return isCollided;
}
void MassSpringSystemSimulator::computeInternalForce(
	const std::vector<PointMass>& points,
	const std::vector<Spring>& springs,
	std::vector<PointMass>& forces)
{
	const int POINT_SIZE = points.size();
	const int SPRING_SIZE = springs.size();

	// force computation
	// calculate internal forces (spring forces)
	for (int sid = 0; sid < SPRING_SIZE; sid++) {
		const Spring& s = springs[sid];

		int pid0 = s._pointID0;
		int pid1 = s._pointID1;

		//if (VALID_POINT(pid0, POINT_SIZE) && VALID_POINT(pid1, POINT_SIZE)) {
			const PointMass& p0 = points[pid0];
			const PointMass& p1 = points[pid1];

			// hook's law
			Vec3 v0 = p0._position - p1._position;
			float length = normalize(v0);
			// forces
			Vec3 f0 = -s._stiffness * (length - s._restLength) * v0
				-s._damping * dot(p0._velocity - p1._velocity, v0) * v0;
			forces[pid0]._force += f0;
			forces[pid1]._force += -f0;
		//}
	}
}

void MassSpringSystemSimulator::dampingForce(const std::vector<PointMass>& points, std::vector<PointMass>& forces)
{
	const int POINT_SIZE = points.size();
	// damping forces
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		const PointMass& p = points[pid];
		forces[pid]._force += -p._damping * p._velocity;
	}
}

void MassSpringSystemSimulator::copyPoints(const std::vector<PointMass>& inPoints, std::vector<PointMass>& outPoints)
{
	const int POINT_SIZE = outPoints.size();
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		outPoints[pid] = inPoints[pid];
	}
}

void MassSpringSystemSimulator::clearPointForce(std::vector<PointMass>& outPoints)
{
	const int POINT_SIZE = outPoints.size();
	// clear forces from last step
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& pOut = outPoints[pid];
		pOut._force = ZERO_VEC;
	}
}

void MassSpringSystemSimulator::applyExternalForceSpring(std::vector<PointMass>& outPoints)
{
	if (!outPoints[_dragPointMassIndex]._isFixed) {
		if (_externalSpringForce < 0)
			_externalSpringForce = 0;

		const float sitffness = _externalSpringForce;
		// hook's law
		Vec3 v0 = outPoints[_dragPointMassIndex]._position - _dragPoint;
		float length = normalize(v0);
		// forces
		outPoints[_dragPointMassIndex]._force += -sitffness * length * v0;
	}
}

void MassSpringSystemSimulator::applyExternalForcePoints(
	std::vector<PointMass>& outPoints, const Vec3& externalForce)
{
	const int POINT_SIZE = outPoints.size();

	// external forces
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = outPoints[pid];
		p._force += externalForce;
	}
}

void MassSpringSystemSimulator::updatePosition(
	const float& timeStep, 
	const std::vector<PointMass>& pointsPos,
	const std::vector<PointMass>& pointsVel,
	std::vector<PointMass>& outPoints)
{
	const int POINT_SIZE = outPoints.size();

	for (int pid = 0; pid < POINT_SIZE; pid++) {
		const PointMass& pPos = pointsPos[pid];
		const PointMass& pVel = pointsVel[pid];
		PointMass& p = outPoints[pid];
		if (!p._isFixed)
		{
			p._position = pPos._position + pVel._velocity * timeStep;
		}
		else {
			p._position = pPos._position;
		}
	}
}

void MassSpringSystemSimulator::updateVelocity(
	const float& timeStep, 
	const std::vector<PointMass>& pointsVel,
	const std::vector<PointMass>& pointsForce,
	std::vector<PointMass>& outPoints)
{
	const int POINT_SIZE = outPoints.size();

	for (int pid = 0; pid < POINT_SIZE; pid++) {
		const PointMass& pVel = pointsVel[pid];
		const PointMass& pForce = pointsForce[pid];
		PointMass& p = outPoints[pid];
		if (!p._isFixed)
		{
			p._acceleration = pForce._force / p._mass;
			p._velocity = pVel._velocity + p._acceleration * timeStep;
		}
		else {
			p._acceleration = ZERO_VEC;
			p._velocity = ZERO_VEC;
		}
	}
}

void MassSpringSystemSimulator::integrateExplicitEuler(
	const float& timeStep, 
	const std::vector<PointMass>& points, const std::vector<Spring>& springs, 
	const Vec3& externalForce, 
	std::vector<PointMass>& outPoints)
{
	copyPoints(points, _tempPoints);

	if (_enableGraviy) {
		applyExternalForcePoints(_tempPoints, _gravity);
	}

	computeInternalForce(_tempPoints, springs, _tempPoints);
	dampingForce(_tempPoints, _tempPoints);

	applyExternalForcePoints(_tempPoints, externalForce);
	if (_enableExternalSpringForce){
		applyExternalForceSpring(_tempPoints);
	}

	updatePosition(timeStep, _tempPoints, _tempPoints, outPoints);
	updateVelocity(timeStep, _tempPoints, _tempPoints, outPoints);

	clearPointForce(outPoints);
	if (_enableCollision) {
		bool hit = collisionResolve(timeStep, outPoints, outPoints);
	}

}

void MassSpringSystemSimulator::integrateMidpoint(
	const float& timeStep, 
	const std::vector<PointMass>& points, const std::vector<Spring>& springs, 
	const Vec3& externalForce, 
	std::vector<PointMass>& outPoints)
{
	std::vector<PointMass> midPoints = points;

	integrateExplicitEuler(timeStep * 0.5,
		points, _springs, this->m_externalForce,
		midPoints);

	copyPoints(midPoints, _tempPoints);

	if (_enableGraviy) {
		applyExternalForcePoints(_tempPoints, _gravity);
	}

	computeInternalForce(_tempPoints, springs, _tempPoints);
	dampingForce(_tempPoints, _tempPoints);

	applyExternalForcePoints(_tempPoints, externalForce);
	if (_enableExternalSpringForce) {
		applyExternalForceSpring(_tempPoints);
	}

	updatePosition(timeStep, points, _tempPoints, outPoints);
	updateVelocity(timeStep, points, _tempPoints, outPoints);

	clearPointForce(outPoints);
	if (_enableCollision) {
		bool hit = collisionResolve(timeStep, outPoints, outPoints);
	}
}

void MassSpringSystemSimulator::integrateLeapfrog(
	const float& timeStep, 
	const std::vector<PointMass>& points, const std::vector<Spring>& springs, 
	const Vec3& externalForce, 
	std::vector<PointMass>& outPoints)
{
	copyPoints(points, _tempPoints);
	clearPointForce(_tempPoints);

	if (_enableGraviy) {
		applyExternalForcePoints(_tempPoints, _gravity);
	}

	computeInternalForce(points, springs, _tempPoints);
	dampingForce(points, _tempPoints);


	applyExternalForcePoints(_tempPoints, externalForce);
	if (_enableExternalSpringForce) {
		applyExternalForceSpring(_tempPoints);
	}
	
	if (_enableCollision) {
		bool hit = collisionResolve(timeStep, _tempPoints, _tempPoints);
	}

	updateVelocity(timeStep, _tempPoints, _tempPoints, outPoints);
	updatePosition(timeStep, _tempPoints, outPoints, outPoints);

}


PointMass::PointMass(float mass, float damping, const Vec3& position, const Vec3& velocity, bool isFixed)
{
	_mass = mass;
	_damping = damping;
	_position = position;
	_velocity = velocity;

	_force = ZERO_VEC;
	_acceleration = ZERO_VEC;

	_isFixed = isFixed;

}

Spring::Spring(int pID0, int pID1, float stiffness, float damping, float restLength)
{
	_pointID0 = pID0;
	_pointID1 = pID1;

	_damping = damping;
	_stiffness = stiffness;
	_restLength = restLength;
}
