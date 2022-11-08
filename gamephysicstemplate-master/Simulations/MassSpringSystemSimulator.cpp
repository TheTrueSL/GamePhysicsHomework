#include "MassSpringSystemSimulator.h"

// Constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	m_iPrintSteps = 0;
	m_fGravity = 0.0;
	DUC = nullptr;

	reset();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo1, Demo2, Demo3, Demo4";
}

void MassSpringSystemSimulator::reset()
{
	m_points.clear();
	m_springs.clear();
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Print Steps", TW_TYPE_INT16, &m_iPrintSteps, "");
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, ""); // m/s2
	const TwType TW_TYPE_INTEGRATOR_MODE = TwDefineEnumFromString("Integrator Mode", "Euler, Leapfrog, Midpoint");
	switch (m_iTestCase)
	{
	case 0: break;
	case 1: break;
	case 2: break;
	case 3: TwAddVarRW(DUC->g_pTweakBar, "Integrator Mode", TW_TYPE_INTEGRATOR_MODE, &m_iIntegrator, "");
		//TODO: ADD MOUSE INTERACTION, GRAVITY, COLLISION
		break;
	default:
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	switch (m_iTestCase)
	{
	case 0:
		cout << "DEMO 1" << endl;
		m_iPrintSteps = 5;
		basicSetup();
		break;
	case 1:
		cout << "DEMO 2" << endl;
		basicSetup();
		break;
	case 2:
		cout << "DEMO 3" << endl;
		m_iIntegrator = 2;
		basicSetup();
		break;
	case 3:
		cout << "DEMO 4" << endl;
		complexSetup();
		break;
	default:
		cout << "Empty Test!" << endl;
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	const int pointsSize = m_points.size();
	const int springsSize = m_springs.size();
	const Vec3 pointScale = 0.05 * Vec3(1, 1, 1);

	// Draw Points
	for (int i = 0; i < pointsSize; i++) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(1, 0, 0));
		DUC->drawSphere(m_points[i].m_vPosition, pointScale);
	}

	// Draw Lines
	DUC->beginLine();
	for (int i = 0; i < springsSize; i++) {
		Spring& s = m_springs[i];
		Point& p1 = m_points[s.m_iPoint1Index];
		Point& p2 = m_points[s.m_iPoint2Index];

		DUC->drawLine(p1.m_vPosition, Vec3(0, 1, 0), p2.m_vPosition, Vec3(1, 1, 1));
	}
	DUC->endLine();
}



void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	const int pointsSize = m_points.size();
	const int springsSize = m_springs.size();

	// Integrate positions and velocity
	switch (m_iIntegrator)
	{
	case EULER:
		integrateEuler(timeStep);
		break;
	case LEAPFROG:
		integrateLeapfrog(timeStep);
		break;
	case MIDPOINT:
		integrateMidpoint(timeStep);
		break;
	default:
		integrateEuler(timeStep);
		break;
	}

	if (m_iPrintSteps > 0) {
		for (int i = 0; i < pointsSize; i++) {
			cout << "p_" << i << " : " << m_points[i].m_vPosition;
			cout << "\tv_" << i << " : " << m_points[i].m_vVelocity << endl;
		}
		cout << endl;
		m_iPrintSteps--;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

void MassSpringSystemSimulator::setGravity(float gravity)
{
	m_fGravity = gravity;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed)
{
	Point p = Point(position, velocity, m_fMass, m_fDamping, isFixed);
	m_points.push_back(p);
	return m_points.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s = Spring(masspoint1, masspoint2, m_fStiffness, initialLength);
	m_springs.push_back(s);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_points[index].m_vPosition;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_points[index].m_vVelocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}

void MassSpringSystemSimulator::clearForces(int index)
{
	m_points[index].m_vForceAccumulator = Vec3(0, 0, 0);
}

void MassSpringSystemSimulator::addExternalForce(int index)
{
	m_points[index].m_vForceAccumulator += m_externalForce;
}

void MassSpringSystemSimulator::addDampingForce(int index)
{
	Point& p = m_points[index];
	p.m_vForceAccumulator -= p.m_fDamping * p.m_vVelocity;
}

void MassSpringSystemSimulator::computeElasticAndAddToEndpoints(int index)
{
	Spring& s = m_springs[index];
	Point& p1 = m_points[s.m_iPoint1Index];
	Point& p2 = m_points[s.m_iPoint2Index];

	float currentLength = norm(p1.m_vPosition - p2.m_vPosition);
	Vec3 direction = p1.m_vPosition - p2.m_vPosition / currentLength;

	float lengthDifference = currentLength - s.m_fInitialLength;
	Vec3 force = -s.m_fStiffness * lengthDifference * direction;
	p1.m_vForceAccumulator += force;
	p2.m_vForceAccumulator += -force;
}

void MassSpringSystemSimulator::updatePosition(int index, float timeStep)
{
	Point& p = m_points[index];
	if (!p.m_bIsFixed) p.m_vPosition += timeStep * p.m_vVelocity;
}

void MassSpringSystemSimulator::updateVelocity(int index, float timeStep)
{
	Point& p = m_points[index];
	if (!p.m_bIsFixed) {
		const Vec3 acceleration = p.m_vForceAccumulator / p.m_fMass + Vec3(0, -m_fGravity, 0);
		p.m_vVelocity += timeStep * acceleration;
	}
	else {
		p.m_vVelocity = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::basicSetup()
{
	setMass(10.0);
	setDampingFactor(0.0);
	int p1Index = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int p2Index = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

	setStiffness(40.0);
	addSpring(p1Index, p2Index, 1.0);

	if (m_iTestCase == 2) m_iIntegrator = MIDPOINT;
	else m_iIntegrator = EULER;
}

void MassSpringSystemSimulator::complexSetup()
{
}

void MassSpringSystemSimulator::integrateEuler(float timeStep)
{
	const int pointsSize = m_points.size();
	const int springsSize = m_springs.size();

	// Compute elastic forces of all springs and add to endpoints
	for (int i = 0; i < springsSize; i++) {
		computeElasticAndAddToEndpoints(i);
	}

	for (int i = 0; i < pointsSize; i++) {
		// Add external force
		addExternalForce(i);
		// Add damping force
		addDampingForce(i);
		// Update positions and velocity
		updatePosition(i, timeStep);
		updateVelocity(i, timeStep);
		// Clear forces
		clearForces(i);
	}
}

void MassSpringSystemSimulator::integrateLeapfrog(float timeStep)
{
}

void MassSpringSystemSimulator::integrateMidpoint(float timeStep)
{
	integrateEuler(timeStep * 0.5);

	const int pointsSize = m_points.size();
	const int springsSize = m_springs.size();

	// Compute elastic forces of all springs and add to endpoints
	for (int i = 0; i < springsSize; i++) {
		computeElasticAndAddToEndpoints(i);
	}

	for (int i = 0; i < pointsSize; i++) {
		// Add external force
		addExternalForce(i);
		// Add damping force
		addDampingForce(i);
		// Update positions and velocity
		updatePosition(i, timeStep);
		updateVelocity(i, timeStep);
		// Clear forces
		clearForces(i);
	}
}

Spring::Spring(int point1Index, int point2Index, float stiffness, float initialLength)
{
	m_iPoint1Index = point1Index;
	m_iPoint2Index = point2Index;
	m_fStiffness = stiffness;
	m_fInitialLength = m_fCurrentLength = initialLength;
}

Point::Point(const Vec3& initialPosition, const Vec3& initialVelocity, float mass, float damping, bool isFixed)
{
	m_vPosition = initialPosition;
	m_vVelocity = initialVelocity;
	m_fMass = mass;
	m_fDamping = damping;
	m_bIsFixed = isFixed;

	m_vForceAccumulator = Vec3(0, 0, 0);
}