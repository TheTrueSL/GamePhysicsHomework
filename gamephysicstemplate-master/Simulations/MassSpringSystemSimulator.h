#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

class PointMass {
	friend class MassSpringSystemSimulator;
public:
	PointMass();
	PointMass(float mass, float damping, const Vec3& position, const Vec3& velocity, bool isFixed);

private:
	bool _isFixed;

	Vec3 _position;
	Vec3 _velocity;
	Vec3 _force;

	float _mass;
	float _damping;
	Vec3 _acceleration;
};

class Spring {
	friend class MassSpringSystemSimulator;
public:
	Spring(int pID0, int pID1, float stiffness, float damping, float restLength);

private:
	int _pointID0;
	int _pointID1;

	float _damping;
	float _stiffness;
	float _restLength;
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
	void setGravity(const Vec3& gravity);
	void setSpringDampingFactor(float damping);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	void loadSimpleSetup();
	void loadComplexSetup();
	void createRope(const Vec3& start, const Vec3& end, int samples);
	void createCloth(const Vec3& start, const Vec3& end, int samples0, int samples1);
	void createBox(const Vec3& center, const float size);
	void createSphere(const Vec3& center, const float radius, int subdivisions);


	bool collisionResolve(
		const float& deltaTime,
		const std::vector<PointMass>& points,
		std::vector<PointMass>& outPoints);

	bool collisionPLane(
		const float& deltaTime,
		const std::vector<PointMass>& points,
		const Vec3& n,
		const float offset,
		const float friction,
		std::vector<PointMass>& outPoints);
	
	void computeInternalForce(
		const std::vector<PointMass>& points,
		const std::vector<Spring>& springs,
		std::vector<PointMass>& outPoints);

	void dampingForce(
		const std::vector<PointMass>& points,
		std::vector<PointMass>& outPoints

	);
	void copyPoints(
		const std::vector<PointMass>& inPoints,
		std::vector<PointMass>& outPoints
	);
	void clearPointForce(
		std::vector<PointMass>& outPoints
	);

	void applyExternalForceSpring(
		std::vector<PointMass>& outPoints
	);

	void applyExternalForcePoints(
		std::vector<PointMass>& outPoints,
		const Vec3& externalForce
	);

	void updatePosition(
		const float& timeStep,
		const std::vector<PointMass>& pointsPos,
		const std::vector<PointMass>& pointsVel,
		std::vector<PointMass>& outPoints
	);

	void updateVelocity(
		const float& timeStep,
		const std::vector<PointMass>& pointsVel,
		const std::vector<PointMass>& pointsForce,
		std::vector<PointMass>& outPoints
	);

	void integrateExplicitEuler(
		const float& timeStep,
		const std::vector<PointMass>& points,
		const std::vector<Spring>& springs,
		const Vec3& externalForce,
		std::vector<PointMass>& outPoints);

	void integrateMidpoint(
		const float& timeStep,
		const std::vector<PointMass>& points,
		const std::vector<Spring>& springs,
		const Vec3& externalForce,
		std::vector<PointMass>& outPoints);

	void integrateLeapfrog(
		const float& timeStep,
		const std::vector<PointMass>& points,
		const std::vector<Spring>& springs,
		const Vec3& externalForce,
		std::vector<PointMass>& outPoints);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	bool _isPressed;
	// events
	void onMouseDown(int x, int y);

private:
	// Simulation
	std::vector<PointMass> _points;
	std::vector<PointMass> _tempPoints;
	std::vector<Spring> _springs;

	int _dragPointMassIndex;
	Vec3 _dragOrigin;
	Vec3 _dragPoint;

	Vec3 _gravity;
	bool _enableGraviy;
	bool _enableCollision;
	bool _enableExternalSpringForce;

	// demo
	int _printSteps = 0;

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fSpringDamping;
	float m_fPointDamping;
	int m_iIntegrator;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif
