#include "MassSpringSystemSimulator.h"

#define VALID_POINT(_i_, _size_) ((_i_) >= 0 && (_i_) < (_size_))

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	DUC         = nullptr;

	m_fMass       = 1;
	m_fStiffness  = 1;
	m_fDamping    = 0;
	m_iIntegrator = 0;

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
		"Euler, Midpoint, Leapfrog");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &this->m_iIntegrator, "");

	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOLCPP, &this->_enableGraviy, "");
	TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_BOOLCPP, &this->_enableCollision, "");

}

void MassSpringSystemSimulator::reset()
{
	_points.clear();
	_springs.clear();

	// UI Attributes
	m_externalForce = Vec3();
	m_mouse = Point2D();
	m_trackmouse = Point2D();
	m_oldtrackmouse = Point2D();
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	const int POINT_SIZE = _points.size();
	const int SPRING_SIZE = _springs.size();

	const float pointScale = 0.01f;
	const Vec3 lineColor0 = Vec3(1., 1., 1.);
	const Vec3 lineColor1 = Vec3(1., 0., 0.);
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = _points[pid];
		this->DUC->drawSphere(p._position, pointScale);
	}
	this->DUC->beginLine();
	for (int sid = 0; sid < SPRING_SIZE; sid++) {
		Spring& s = _springs[sid];
		int pid0 = s._pointID0;
		int pid1 = s._pointID1; 
		if (VALID_POINT(pid0, POINT_SIZE) && VALID_POINT(pid1, POINT_SIZE)) {
			PointMass& p0 = _points[pid0];
			PointMass& p1 = _points[pid1];
			this->DUC->drawLine(p0._position, lineColor0, p1._position, lineColor1);
		}
	}
	this->DUC->endLine();

	//this->DUC->DrawTriangleUsingShaders();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	std::cout << "Case Changed : " << testCase << std::endl;
	switch (testCase)
	{
	case 0:
		printSteps = 1;
		_enableCollision = false;
		_enableGraviy = false;
		loadSimpleSetup();
		break;
	case 1:
		printSteps = -1;
		_enableCollision = false;
		_enableGraviy = false;
		loadSimpleSetup();
		break;
	case 2:
		printSteps = -1;
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

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (printSteps == 0)
		return;

	if (m_iIntegrator == EULER) {
		integrateExplicitEuler(timeStep, _points, _springs, this->m_externalForce, _points);
	}
	else if (m_iIntegrator == MIDPOINT) {
		integrateMidpoint(timeStep, _points, _springs, this->m_externalForce, _points);
	}
	else if (m_iIntegrator == LEAPFROG) {
		integrateLeapfrog(timeStep, _points, _springs, this->m_externalForce, _points);
	}
	else {
		integrateExplicitEuler(timeStep, _points, _springs, this->m_externalForce, _points);
	}

	if (printSteps > 0) {
		for (int i = 0; i < _points.size(); i++) {
			std::cout << "p " << i << ": " << _points[i]._position << std::endl;
			std::cout << "v " << i << ": " << _points[i]._velocity << std::endl;
		}
		printSteps -= 1;
	}
	this->m_externalForce = Vec3(0, 0, 0);
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
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
	this->m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int point_idx = -1;

	float mass = this->m_fMass;
	float damping = this->m_fDamping;

	this->_points.push_back(PointMass(mass, damping, position, Velocity, isFixed));
	
	point_idx = this->_points.size() - 1;

	return point_idx;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	float stiffness = this->m_fStiffness;

	this->_springs.push_back(Spring(masspoint1, masspoint2, stiffness, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return this->_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return this->_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return this->_points[index]._position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return this->_points[index]._velocity;
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

	float damping = 0.01;
	Vec3 gravity(0, -5, 0);

	this->setDampingFactor(damping);
	this->setGravity(gravity);
	// at least 10 points and springs
	// ground or wall collisions
}

bool MassSpringSystemSimulator::collisionResolve(
	const std::vector<PointMass>& points,
	std::vector<PointMass>& outPoints)
{
	bool isCollided = false;
	
	const int POINT_SIZE = points.size();
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		const PointMass& pIn = points[pid];
		PointMass& pOut = outPoints[pid];
		pOut = pIn;
		if (!pOut._isFixed && pOut._position.z < 0) {
			pOut._position.z = 0;
			isCollided = true;
		}
	}

	return isCollided;
}
void MassSpringSystemSimulator::computeInternalForce(
	const std::vector<PointMass>& points,
	const std::vector<Spring>& springs,
	std::vector<PointMass>& outPoints)
{
	const int POINT_SIZE = points.size();
	const int SPRING_SIZE = points.size();

	for (int pid = 0; pid < POINT_SIZE; pid++) {
		outPoints[pid] = points[pid];
	}

	// force computation
	// calculate internal forces (spring forces)
	for (int sid = 0; sid < SPRING_SIZE; sid++) {
		const Spring& s = springs[sid];

		int pid0 = s._pointID0;
		int pid1 = s._pointID1;

		if (VALID_POINT(pid0, POINT_SIZE) && VALID_POINT(pid1, POINT_SIZE)) {
			PointMass& p0 = outPoints[pid0];
			PointMass& p1 = outPoints[pid1];

			// hook's law
			Vec3 v0 = p0._position - p1._position;
			float length = norm(v0);
			// forces
			Vec3 f0 = -s._stiffness * (length - s._restLength) * (v0 / (length + 1e-8));
			p0._force += f0;
			p1._force += -f0;
		}
	}

	// damping forces
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& p = outPoints[pid];
		p._force -= p._damping * p._velocity;
	}
}

void MassSpringSystemSimulator::clearPointForce(std::vector<PointMass>& outPoints)
{
	const int POINT_SIZE = outPoints.size();
	// clear forces from last step
	for (int pid = 0; pid < POINT_SIZE; pid++) {
		PointMass& pOut = outPoints[pid];
		pOut._force = Vec3(0, 0, 0);
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
			p._acceleration = Vec3(0, 0, 0);
			p._velocity = Vec3(0, 0, 0);
		}
	}
}

void MassSpringSystemSimulator::integrateExplicitEuler(
	const float& timeStep, 
	const std::vector<PointMass>& points, const std::vector<Spring>& springs, 
	const Vec3& externalForce, 
	std::vector<PointMass>& outPoints)
{
	computeInternalForce(points, springs, outPoints);
	applyExternalForcePoints(outPoints, externalForce);
	if (_enableGraviy) {
		applyExternalForcePoints(outPoints, _gravity);
	}
	updatePosition(timeStep, points, outPoints, outPoints);
	updateVelocity(timeStep, points, outPoints, outPoints);
	clearPointForce(outPoints);

	if (_enableCollision) {
		bool hit = collisionResolve(outPoints, outPoints);
	}
}

void MassSpringSystemSimulator::integrateMidpoint(
	const float& timeStep, 
	const std::vector<PointMass>& points, const std::vector<Spring>& springs, 
	const Vec3& externalForce, 
	std::vector<PointMass>& outPoints)
{
	std::vector<PointMass> pointsMid = points;

	computeInternalForce(points, springs, pointsMid);
	applyExternalForcePoints(pointsMid, externalForce);
	if (_enableGraviy) {
		applyExternalForcePoints(pointsMid, _gravity);
	}
	updateVelocity(timeStep * 0.5, points, pointsMid, pointsMid);
	updatePosition(timeStep * 0.5, points, pointsMid, pointsMid);
	clearPointForce(pointsMid);

	if (_enableCollision) {
		bool hit = collisionResolve(pointsMid, pointsMid);
	}

	computeInternalForce(pointsMid, springs, pointsMid);
	applyExternalForcePoints(pointsMid, externalForce);
	if (_enableGraviy) {
		applyExternalForcePoints(pointsMid, _gravity);
	}
	updateVelocity(timeStep, points, pointsMid, outPoints);
	updatePosition(timeStep, points, pointsMid, outPoints);

	clearPointForce(outPoints);

	if (_enableCollision) {
		bool hit = collisionResolve(outPoints, outPoints);
	}
}

void MassSpringSystemSimulator::integrateLeapfrog(
	const float& timeStep, 
	const std::vector<PointMass>& points, const std::vector<Spring>& springs, 
	const Vec3& externalForce, 
	std::vector<PointMass>& outPoints)
{
	std::vector<PointMass> pointsMid = points;
	// a_0
	computeInternalForce(points, springs, pointsMid);
	applyExternalForcePoints(pointsMid, externalForce);
	if (_enableGraviy) {
		applyExternalForcePoints(pointsMid, _gravity);
	}
	// v_1/2 = v_0 + a_0 * 0.5 * h
	updateVelocity(timeStep * 0.5, points, pointsMid, pointsMid);
	// x_1 = x_0 + v_1/2 * h
	updatePosition(timeStep, points, pointsMid, outPoints);

	if (_enableCollision) {
		bool hit = collisionResolve(outPoints, outPoints);
	}

	// a_1
	computeInternalForce(outPoints, springs, outPoints);
	if (_enableGraviy) {
		applyExternalForcePoints(pointsMid, _gravity);
	}
	// v_1 = v_1/2 + a_1 * 0.5 * h
	updateVelocity(timeStep * 0.5, pointsMid, outPoints, outPoints);
	
	clearPointForce(outPoints);
}

PointMass::PointMass(float mass, float damping, const Vec3& position, const Vec3& velocity, bool isFixed)
{
	_mass = mass;
	_damping = damping;
	_position = position;
	_velocity = velocity;

	_force = Vec3(0,0,0);
	_acceleration = Vec3(0, 0, 0);

	_isFixed = isFixed;

}

Spring::Spring(int pID0, int pID1, float stiffness, float restLength)
{
	_pointID0 = pID0;
	_pointID1 = pID1;

	_stiffness = stiffness;
	_restLength = restLength;
}

Spring::Spring(int pID0, int pID1, float stiffness, float restLength, float length)
	:Spring(pID0, pID1, stiffness, restLength)
{
}
