#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class Spring {
	friend class MassSpringSystemSimulator;
public:
	Spring(int point1Index, int point2Index, float stiffness, float initialLength);
private:
	int m_iPoint1Index;
	int m_iPoint2Index;
	float m_fStiffness;
	float m_fInitialLength;
	float m_fCurrentLength;
};

class Point {
	friend class MassSpringSystemSimulator;
public:
	Point(const Vec3& initialPosition, const Vec3& initialVelocity, float mass, float damping, bool isFixed);
private:
	Vec3 m_vPosition;
	Vec3 m_vVelocity;
	Vec3 m_vForceAccumulator;
	float m_fMass;
	float m_fDamping;
	bool m_bIsFixed;
};
class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
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
	void setGravity(float gravity);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	void clearForces(int index);
	void addExternalForce(int index);
	void addDampingForce(int index);
	void computeElasticAndAddToEndpoints(int index);
	void updatePosition(int index, float timeStep);
	void updateVelocity(int index, float timeStep);

	// Demo Setups
	void basicSetup();
	void complexSetup();

	// Integrations
	void integrateEuler(float timeStep);
	void integrateLeapfrog(float timeStep);
	void integrateMidpoint(float timeStep);

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
	float m_fGravity;

	int m_iPrintSteps;

	vector<Point> m_points;
	vector<Spring> m_springs;
	
	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif