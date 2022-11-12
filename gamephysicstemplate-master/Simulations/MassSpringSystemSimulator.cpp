#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_iIntegrator = 0;
	m_steps = 0;
	reset();
}

MassSpringSystemSimulator::~MassSpringSystemSimulator()
{
	for (int i = 0; i < slist.size(); i++) {
		delete slist[i];
	}
	for (int i = 0; i < plist.size(); i++) {
		delete plist[i];
	}
	slist.clear();
	plist.clear();
	forcesOnPointMasses.clear();
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Simple Demo, Complex Demo";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator Type",
		"Euler, Leapfrog, Midpoint");
	switch (m_iTestCase)
	{
	case 0:
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &this->m_iIntegrator, "");
		break;
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &this->m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=0.01 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.01 step=0.01");
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset() {
	
	for (int i = 0; i < slist.size(); i++) {
		delete slist[i];
	}
	for (int i = 0; i < plist.size(); i++) {
		delete plist[i];
	}

	slist.clear();
	plist.clear();
	forcesOnPointMasses.clear();
	accPrevList.clear();
	posPrevList.clear();
	velPrevList.clear();

	m_externalForce = (0, 0, 0);
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	const Vec3 pointScale = 0.05 * Vec3(1, 1, 1);

	// Draw Point Masses
	for (int i = 0; i < plist.size(); i++) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(1, 0, 0));
		DUC->drawSphere((*plist[i]).getPointPosition(), pointScale);
	}

	// Draw Springs
	DUC->beginLine();
	for (int i = 0; i < slist.size(); i++) {
		Spring s = (*slist[i]);
		int p0 = s.getFirstPointInd();
		int p1 = s.getSecondPointInd();
		PointMass pm0 = *plist[p0];
		PointMass pm1 = *plist[p1];

		DUC->drawLine(pm0.getPointPosition(), Vec3(0, 1, 0), pm1.getPointPosition(), Vec3(1, 1, 1));
	}
	DUC->endLine();
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	reset();
	switch (m_iTestCase)
	{
	case 0:
		cout << "SIMPLE DEMO Information\n";
		loadSimpleDemo();
		setIntegrator(0);
		break;
	case 1:
		cout << "COMPLEX DEMO Information\n";
		loadComplexDemo();
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (m_steps == 0)
		return;
	// Handle different integration methods
	switch (m_iIntegrator)
	{
	case 0:
		integrateEuler(timeStep);
		break;
	case 1:
		break;
	case 2:
		//integrateMidpoint(timeStep);
		break;
	default:
		integrateEuler(timeStep);
		break;
	}

	if (m_steps > 0) {
		for (int i = 0; i < plist.size(); i++) {
			cout << "p_" << i << " : " << (*plist[i]).getPointPosition();
			cout << "\tv_" << i << " : " << (*plist[i]).getPointVelocity() << endl;
		}
		cout << endl;
		m_steps--;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	PointMass *p = new PointMass(m_fMass, position, Velocity, isFixed);
	plist.push_back(p);
	posPrevList.push_back(position);
	velPrevList.push_back(Velocity);
	forcesOnPointMasses.push_back((0.0, 0.0, 0.0));
	accPrevList.push_back((0.0, 0.0, 0.0));
	return plist.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring* s = new Spring(m_fStiffness, m_fDamping, initialLength, masspoint1, masspoint2);
	slist.push_back(s);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return plist.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return slist.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return (*plist[index]).getPointPosition();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return (*plist[index]).getPointVelocity();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {

}

//demo functions

void MassSpringSystemSimulator::loadSimpleDemo() {
	reset();
	//Initialize point masses
	Vec3 p0(0.0, 0.0, 0.0);
	Vec3 p1(0.0, 2.0, 0.0);
	Vec3 v0(-1.0, 0.0, 0.0);
	Vec3 v1(1.0, 0.0, 0.0);
	
	setStiffness(40.0);

	//Step size can be changed from here!
	setStep(1000);
	setMass(10.0);
	setDampingFactor(0);
	addMassPoint(p0, v0, false);
	addMassPoint(p1, v1, false);
	addSpring(0.0, 1.0, 1.0);
}

void MassSpringSystemSimulator::loadComplexDemo() {

}

void MassSpringSystemSimulator::setStep(int stepS)
{
	m_steps = stepS;
}

// Internal force calculations for all points in simulation - Hooke's Law
void MassSpringSystemSimulator::computeElasticForces() {
	//Compute forces for all springs
	for (int i = 0; i < slist.size(); i++) {
		int p0 = (*slist[i]).getFirstPointInd();
		int p1 = (*slist[i]).getSecondPointInd();
		
		PointMass point0 = (*plist[p0]);
		PointMass point1 = (*plist[p1]);
		
		//Apply Hooke's Law
		float pointLength = norm(point0.getPointPosition() - point1.getPointPosition());
		Vec3 direction = (point0.getPointPosition() - point1.getPointPosition()) / pointLength;
		Vec3 force = -m_fStiffness * (pointLength - (*slist[i]).getRestLength()) * direction;

		//Accumulate applied forces to each point mass and save it
		forcesOnPointMasses[p0] += force;
		forcesOnPointMasses[p1] += - force;
		cout << "checkpointsss" << endl;
	}
}

//Update velocities of present point masses
void MassSpringSystemSimulator::updateVelocityEuler(float timeStep) {
	
	Vec3 acc = (0.0, 0.0, 0.0);
	Vec3 newVelocity = (0.0, 0.0, 0.0);
	for (int i = 0; i < plist.size(); i++)
	{
		//Calculate acceleration and update velocity
		PointMass p = (*plist[i]);
		if (!p._isFixed()) {
			acc = (forcesOnPointMasses[i] / m_fMass);
			newVelocity = velPrevList[i] + (timeStep * acc);
			accPrevList[i] = acc;
			(*plist[i]).setPointVelocity(newVelocity);
			velPrevList[i] = newVelocity;
		}
		else {
			(*plist[i]).setPointVelocity((0.0, 0.0, 0.0));
		}
	}
}

//Update positions of present point masses
void MassSpringSystemSimulator::updatePositionEuler(float timeStep) {
	Vec3 newPos = (0.0, 0.0, 0.0);
	for (int i = 0; i < plist.size(); i++)
	{
		//Update new pos using old pos
		PointMass p = (*plist[i]);
		if (!p._isFixed()) {
			newPos = posPrevList[i] + (timeStep * velPrevList[i]);
			posPrevList[i] = newPos;
			(*plist[i]).setPointPosition(newPos);
		}
	}
}

// Integration Methods
void MassSpringSystemSimulator::integrateEuler(float timestep) {
	computeElasticForces();
	updatePositionEuler(timestep);
	updateVelocityEuler(timestep);

	//Reset forces for next time step
	for (int i = 0; i < forcesOnPointMasses.size(); i++) {
		forcesOnPointMasses[i] = (0.0, 0.0, 0.0);
	}
}

void MassSpringSystemSimulator::integrateMidpoint(float timestep) {

}

void MassSpringSystemSimulator::integrateLeapFrog(float timestep) {

}


