#include "MassSpringSystemSimulator.h"


// Constructors
MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	setMass(10);
	setStiffness(1);
	setDampingFactor(1);
	m_iIntegrator = 0;
	setGravity(10);
	DUC = nullptr;
	reset();
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo1, Demo2, Demo3, Demo4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator Type",
		"Euler, Leapfrog, Midpoint");
	TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &this->m_iIntegrator, "");
}

void MassSpringSystemSimulator::reset()
{
	_masspoints.clear();
	_springs.clear();

}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
		const int pointsSize = getNumberOfMassPoints();
		const int springsSize = getNumberOfSprings();
		const Vec3 pointScale = 0.05 * Vec3(1, 1, 1);

		
		// Draw Points
		for (int i = 0; i < pointsSize; i++) {
			DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(1, 0, 0));
			DUC->drawSphere(_masspoints[i]._position, pointScale);
		}

		// Draw Lines
		DUC->beginLine();
		for (int i = 0; i < springsSize; i++) {
			Spring& s = _springs[i];
			Masspoint& p1 = _masspoints[s._pointAId];
			Masspoint& p2 = _masspoints[s._pointBId];

			DUC->drawLine(p1._position, Vec3(0, 1, 0), p2._position, Vec3(1, 1, 1));
		}
		DUC->endLine();
		
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
	switch (testCase) 
	{
	case 0:
		setUpOne();
		break;
	case 1:
		setUpTwo();
		break;
	case 2:
		setUpThree();
		break;
	case 3:
		setUpThree();
		break;
	default:
		cout << "No Demo found\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	switch (m_iIntegrator)
	{
	case EULER:
		integrateEuler(timeStep);
		break;
	default:
		cout << "ERROR\n";
		break;
	}
}

void MassSpringSystemSimulator::integrateEuler(float timestep)
{
	for(int i = 0; i < getNumberOfSprings(); i++)
	{
		hookesLawForces(i);
	}
	for (int j = 0; j < getNumberOfMassPoints(); j++)
	{
		
		updatePosition(j, timestep);
		updateVelocity(j, timestep);
		_masspoints[j]._force = Vec3(0, 0, 0);
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

// Specific Functions
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
	_gravity = Vec3(0,-gravity,0);
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	_masspoints.push_back(Masspoint(position, Velocity, m_fMass, m_fDamping, isFixed));
	return getNumberOfMassPoints() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring newSpring = Spring(masspoint1, masspoint2, m_fStiffness, initialLength);
	_springs.push_back(newSpring);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return _masspoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return _springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return _masspoints[index]._position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return _masspoints[index]._velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

void MassSpringSystemSimulator::hookesLawForces(int springIndex)
{
	Spring& spring = _springs[springIndex];
	Masspoint& pointA = _masspoints[spring._pointAId];
	Masspoint& pointB = _masspoints[spring._pointBId];

	float currentLength = norm(pointA._position - pointB._position);
	Vec3 direction = (pointA._position - pointB._position) / currentLength;

	Vec3 newForce = -spring._stiffness * (currentLength - spring._springInitialLength) * direction;
	pointA._force += newForce;
	pointB._force += newForce;


}

void MassSpringSystemSimulator::updatePosition(int pointIndex, float timeStep)
{
	Masspoint& point = _masspoints[pointIndex];
	if (point._isFixed) return;
	point._position += timeStep * point._velocity;
}

void MassSpringSystemSimulator::updateVelocity(int pointIndex, float timeStep)
{
	Masspoint& point = _masspoints[pointIndex];
	if (point._isFixed) return;
	
	Vec3 acceleration = (point._force / point._mass) + _gravity;
	cout << "I was reached" << _gravity << acceleration <<"\n";
	point._velocity += timeStep * acceleration;
}

void MassSpringSystemSimulator::setUpOne()
{
	setMass(10);
	setDampingFactor(0);
	setGravity(1);
	setStiffness(40);
	int indexA = addMassPoint(Vec3(0,0,0),Vec3(-1,0,0),false);
	int indexB = addMassPoint(Vec3(0, 2, 0),Vec3(1, 0, 0), true);
	addSpring(indexA, indexB, 1.0);
	m_iIntegrator = EULER;
}

void MassSpringSystemSimulator::setUpTwo()
{
	setMass(10);
	setDampingFactor(0);
	setGravity(1);
	setStiffness(40);
	int indexA = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int indexB = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), true);
	addSpring(indexA, indexB, 1.0);
	m_iIntegrator = EULER;
}

void MassSpringSystemSimulator::setUpThree()
{
	setMass(10);
	setDampingFactor(0);
	setGravity(1);
	setStiffness(40);
	int indexA = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int indexB = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), true);
	addSpring(indexA, indexB, 1.0);
	m_iIntegrator = EULER;
}

void MassSpringSystemSimulator::setUpFour()
{
	setMass(10);
	setDampingFactor(0);
	setGravity(1);
	setStiffness(40);
	int indexA = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int indexB = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), true);
	addSpring(indexA, indexB, 1.0);
	m_iIntegrator = EULER;
}

