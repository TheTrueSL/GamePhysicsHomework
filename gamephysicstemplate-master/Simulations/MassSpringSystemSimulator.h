#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class Masspoint {
	friend class MassSpringSystemSimulator;
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
	friend class MassSpringSystemSimulator;
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

class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	//Added functions
	void integrateEuler(float timestep);
	void setGravity(float gravity);
	void hookesLawForces(int springIndex);
	void updatePosition(int pointIndex, float timeStep);
	void updateVelocity(int pointIndex, float timeStep);
	void setUpOne();
	void setUpTwo();
	void setUpThree();
	void setUpFour();

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	//Additional Attributes
	Vec3 _gravity;
	vector<Masspoint> _masspoints;
	vector<Spring> _springs;
};
#endif