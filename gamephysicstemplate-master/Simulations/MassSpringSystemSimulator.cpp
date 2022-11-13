#include "MassSpringSystemSimulator.h"

const float Y_GROUND_LEVEL = -1.0;
int CLOTH_SIZE = 3;

void MassSpringSystemSimulator::setGravity(float gravity)
{
	m_fGravity = gravity;
}

void MassSpringSystemSimulator::clearForces(int index)
{
	m_points[index].m_vForceAccumulator = Vec3(0, 0, 0);
}

void MassSpringSystemSimulator::addExternalForce(int index)
{
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
	if (m_iPrintSteps > 0) cout << "p1Pos : " << p1.m_vPosition << "  p2Pos : " << p2.m_vPosition << endl;
	if (m_iPrintSteps > 0) cout << "Length btw. " << s.m_iPoint1Index << ":" << s.m_iPoint2Index << " = " << lengthDifference << endl;
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

bool MassSpringSystemSimulator::checkCollision(int index)
{
	Vec3 pos = m_points[index].m_vPosition;
	return pos.Y <= Y_GROUND_LEVEL;
}

void MassSpringSystemSimulator::handleCollisions()
{
	for (int i = 0; i < m_points.size(); i++) {
		if (!checkCollision(i)) continue;
		Point& p = m_points[i];
		Vec3 v = p.m_vVelocity;
		Vec3 pos = p.m_vPosition;
		Vec3 newV = Vec3(v.X, -v.Y, v.Z);
		Vec3 newPos = Vec3(pos.Y, Y_GROUND_LEVEL, pos.Z);
		p.m_vVelocity = newV;
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
	const float UNIT_LENGTH = 1;
	vector<vector<int>> p;
	setMass(1.0);
	setDampingFactor(1.0);
	for (int x = 0; x < CLOTH_SIZE; x++) {
		vector<int> row;
		for (int z = 0; z < CLOTH_SIZE; z++) {
			bool isFixed = false;
			Vec3 v = Vec3(0, 0, 0);
			float y = 0.5;
			if ((x == 0 && z == 0) || (x == CLOTH_SIZE - 1 && z == 0) || (x == 0 && z == CLOTH_SIZE - 1) || (x == CLOTH_SIZE - 1 && z == CLOTH_SIZE - 1)) {
				isFixed = true;
				// y = 0.5;
				if (x == 0 && z == 0) {
					//isFixed = false;
					// v = Vec3(-10, 10, -10);
				}
			}
			row.push_back(addMassPoint(Vec3(x * UNIT_LENGTH - (CLOTH_SIZE / 2) * UNIT_LENGTH, y, z * UNIT_LENGTH - (CLOTH_SIZE / 2) * UNIT_LENGTH), v, isFixed));
		}
		p.push_back(row);
	}

	// Structural
	for (int x = 0; x < CLOTH_SIZE; x++) {
		for (int z = 0; z < CLOTH_SIZE; z++) {
			setStiffness(1000.0);
			//STRUCTURAL
			if (z + 1 < CLOTH_SIZE) addSpring(p[x][z], p[x][z + 1], UNIT_LENGTH);
			if (x + 1 < CLOTH_SIZE) addSpring(p[x][z], p[x + 1][z], UNIT_LENGTH);
			
			setStiffness(1000.0);
			//SHEAR
			if (x + 1 < CLOTH_SIZE && z + 1 < CLOTH_SIZE) addSpring(p[x][z], p[x + 1][z + 1], sqrt(pow(UNIT_LENGTH, 2) + pow(UNIT_LENGTH, 2)));
			if (x - 1 >= 0 && z + 1 < CLOTH_SIZE) addSpring(p[x][z], p[x - 1][z + 1], sqrt(pow(UNIT_LENGTH, 2) + pow(UNIT_LENGTH, 2)));

			setStiffness(1000.0);
			//FLEXION
			if (x + 2 < CLOTH_SIZE) addSpring(p[x][z], p[x + 2][z], UNIT_LENGTH * 2.0);
			if (z + 2 < CLOTH_SIZE) addSpring(p[x][z], p[x][z + 2], UNIT_LENGTH * 2.0);

		}
	}

	cout << "Structural: " << UNIT_LENGTH << endl;
	cout << "SHEAR: " << sqrt(pow(UNIT_LENGTH, 2) + pow(UNIT_LENGTH, 2)) << endl;
	cout << "FLEXION: " << UNIT_LENGTH * 2.0 << endl;
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
		// addExternalForce(sim, i);
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
	const int pointsSize = m_points.size();
	const int springsSize = m_springs.size();

	// Compute elastic forces of all springs and add to endpoints
	for (int i = 0; i < springsSize; i++) {
		computeElasticAndAddToEndpoints(i);
	}

	// Compute xtmp at t+h/2 based on v(t)
	vector<Vec3> xtmp;
	vector<Vec3> vtmp;
	vector<Vec3> originalX;
	for (int i = 0; i < pointsSize; i++) {
		Point& p = m_points[i];
		if (!p.m_bIsFixed) xtmp.push_back(p.m_vPosition + timeStep * 0.5 * p.m_vVelocity);
		else xtmp.push_back(p.m_vPosition);

		// Compute a(t) (based on forces)
		// addExternalForce(sim, i);
		addDampingForce(i);
		Vec3 acceleration;
		if (!p.m_bIsFixed) acceleration = p.m_vForceAccumulator / p.m_fMass + Vec3(0, -m_fGravity, 0);
		else acceleration = Vec3(0, 0, 0);

		// Compute vtmp at t+h/2 based on a(t)
		vtmp.push_back(p.m_vVelocity + 0.5 * timeStep * acceleration);

		// Compute x at t+h
		p.m_vPosition += timeStep * vtmp[i];
		originalX.push_back(p.m_vPosition);
		p.m_vPosition = xtmp[i];
		clearForces(i);
	}

	// Compute a at t+h/2 based on xtmp and vtmp (recalculate elastics at t+h/2 based on xtmp and vtmp)
	for (int i = 0; i < springsSize; i++) {
		computeElasticAndAddToEndpoints(i);
	}

	// Compute v at t+h
	for (int i = 0; i < pointsSize; i++) {
		Point& p = m_points[i];

		// addExternalForce(sim, i);
		addDampingForce(i);
		Vec3 acceleration;
		if (!p.m_bIsFixed) acceleration = p.m_vForceAccumulator / p.m_fMass + Vec3(0, -m_fGravity, 0);
		else acceleration = Vec3(0, 0, 0);
		p.m_vPosition = originalX[i];
		p.m_vVelocity += timeStep * acceleration;
		clearForces(i);
	}
}

// Constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	m_iPrintSteps = 0;
	m_fGravity = 0.0;
	m_iIntegrator = 0;
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
	case 3: 
		TwAddVarRW(DUC->g_pTweakBar, "Integrator Mode", TW_TYPE_INTEGRATOR_MODE, &m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "SIZE", TW_TYPE_INT16, &CLOTH_SIZE, "");
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
	handleCollisions();
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