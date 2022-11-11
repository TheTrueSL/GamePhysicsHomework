#include "MassSpringSystemSimulator.h"

class Masspoint {
public:
	Vec3 _position;
	Vec3 _velocity;
	Vec3 _force;
	float _mass;
	float _damping;
	bool _isFixed = fixed;

	Masspoint(const Vec3 initialPosition, const Vec3 initialVelocity, float mass, float damping, bool fixed)
	{
		_position = initialPosition;
		_velocity = initialVelocity;
		_mass = mass;
		_damping = damping;
		_isFixed = fixed;
		_force = (0, 0, 0);
	}

};

class Spring {
public:
	int _pointAId;
	int _pointBId;
	float _stiffness;
	float _springInitialLength;
	float _springCurrentLength;

	Spring(int pointAId, int pointBId, float stiffness, float length)
	{
		_pointAId = pointAId;
		_pointBId = pointBId;
		_stiffness = stiffness;
		_springInitialLength = length;
		_springCurrentLength = length;
	}

};

//Additional Attributes
vector<Masspoint> _masspoints;
vector<Spring> _springs;
Vec3 _gravity;

// Constructors
MassSpringSystemSimulator::MassSpringSystemSimulator()
{	
	m_fMass = 1;
	m_fStiffness = 1;
	m_fDamping = 0;
	m_iIntegrator = 0;
	_gravity = Vec3(0, -1, 0);
	reset();
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo1, Demo2, Demo3, Demo4";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
}

void MassSpringSystemSimulator::reset()
{
	_masspoints.clear();
	_springs.clear();

}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	reset();
	switch (testCase) 
	{
	case 0:
		cout << "Starting Demo 1\n";
		setUpOne();
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
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
	for(int i = 0; i < _springs.size(); i++)
	{
		hookesLawForces(i);
	}
	for (int j = 0; j < _masspoints.size(); j++)
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
	return _masspoints.size() - 1;
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
	cout << "I am at" << point._position << "n";
}

void MassSpringSystemSimulator::updateVelocity(int pointIndex, float timeStep)
{
	Masspoint& point = _masspoints[pointIndex];
	if (point._isFixed) return;
	Vec3 acceleration = (point._force / point._mass) + _gravity;
	point._velocity += timeStep * acceleration;
}

void MassSpringSystemSimulator::setUpOne()
{
	setMass(10);
	setDampingFactor(0);
	setGravity(0);
	setStiffness(40);
	int indexA = addMassPoint(Vec3(0,0,0),Vec3(-1,0,0),false);
	int indexB = addMassPoint(Vec3(0, 2, 0),Vec3(1, 0, 0), false);
	addSpring(indexA, indexB, 1.0);
	m_iIntegrator = EULER;
}

