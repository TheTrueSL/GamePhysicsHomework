#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	m_fMass = 0;
	m_fStiffness = 0;
	m_fDamping = 0;
	m_iIntegrator = -1;
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
}

const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo 1,Demo 2,Demo 3,Demo 4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator Type",
		"Euler, Leapfrog, Midpoint");
	switch (m_iTestCase)
	{
	case 0:break;
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &this->m_iIntegrator, "");
	case 1:break;
	case 2:break;
	case 3:
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
		cout << "DEMO 1 Information\n";
		loadDemo1Setup();
		break;
	case 1:
		cout << "DEMO 2 Information\n";
		setIntegrator(0);
		loadDemo2Setup();
		break;
	case 2:
		cout << "DEMO 3 Information\n";
		setIntegrator(2);
		loadDemo3Setup();
		break;
	case 3:
		cout << "DEMO 4 Information\n";
		loadDemo4Setup();
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
	// Handle different integration methods
	switch (m_iIntegrator)
	{
	case 0:
		//integrateEuler(timeStep);
		break;
	case 1:
		//integrateLeapfrog(timeStep);
		break;
	case 2:
		//integrateMidpoint(timeStep);
		break;
	default:
		//integrateEuler(timeStep);
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

void MassSpringSystemSimulator::loadDemo1Setup() {
	reset();

	//Initialize point masses
	Vec3 p0(0, 0, 0);
	Vec3 p1(0, 2, 0);
	Vec3 v0(-1, 0, 0);
	Vec3 v1(1, 0, 0);
	
	setStiffness(40);
	setStep(1);
	setDampingFactor(0);
	addMassPoint(p0, v0, false);
	addMassPoint(p1, v1, false);
	addSpring(0, 1, 1);
}

void MassSpringSystemSimulator::loadDemo2Setup() {

}

void MassSpringSystemSimulator::loadDemo3Setup() {

}

void MassSpringSystemSimulator::loadDemo4Setup() {

}

void MassSpringSystemSimulator::setStep(int stepS)
{
	m_steps = stepS;
}

void MassSpringSystemSimulator::computeElasticForces() {

}

// Integration Methods


