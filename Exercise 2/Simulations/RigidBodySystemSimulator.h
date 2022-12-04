#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
#include "SimulationObjects.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	enum class Integrator {
		EULER,
		LEAPFROG
	};
	class Bullet {
	public:
		Bullet(int index_, float lifeTime_);
		int index;
		float lifeTime;
	};
	std::vector<Vec3> contacts;

	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Scene Creation
	void loadSimpleSetup();
	void loadTwoBodySetup();
	void loadComplexSetup();
	void loadTestSetup();
	void createRope(float mass, float damping, float stiffness, const Vec3& start, const Vec3& end, int samples);
	void createCloth(float mass, float damping, float stiffness, const Vec3& start, const Vec3& end, int samples0, int samples1);
	void createBox(float mass, float damping, float stiffness, const Vec3& center, const float size);
	// 
	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	int addMassPoint(float mass, Vec3 position, Vec3 Velocity, bool isFixed=false, float radius=0.01);
	void addSpring(float damping, float stiffness, int masspoint1, int masspoint2);
	void addSpring(float damping, float stiffness, int masspoint1, int masspoint2, float initialLength);
	void addRigidBody(Vec3 position, Vec3 size, int mass, bool isFixed=false);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// mouse state for click detection
	bool _isPressed = false;
	time_t _pressedTimer;

	// events
	void onMouseDown(int x, int y);
	void onMouseDouble(int x, int y);

	void selectDragPoint(int x, int y, int& outType, int& outIndex);
	void shootPointMass(int x, int y);

	bool _isDragging = false;
	int _dragIndex = -1;
	int _dragType = -1;
	Vec3 _dragOrigin;
	Vec3 _dragObjectPoint;
	Vec3 _dragMousePoint;

	Vec3 _gravity;
	float _linearDamping;
	float _angularDamping;
	float _externalSpringForce;

	bool _pause;
	bool _enableShoot;
	bool _enableGraviy;
	bool _enableCollision;
	bool _enableWallCollision;
	bool _enableFloorCollision;

	Integrator _integrator = Integrator::EULER;

	// demo
	int _printSteps = 0;

private:
	// Simulation
	std::vector<PointMass> _points;
	std::vector<Spring> _springs;
	std::vector<RigidBody> _rigidbodies;
	std::vector<Bullet> _bullets;
	int bulletStartIndex;
	PointMass _pendingBullet;
	bool _fillBullet;

	float _passedTime;

	void integrateExplicitEuler(const float& timeStep);
	void integrateLeapfrog(const float& timeStep);
	void clearForce();
	void computeSpringForce();
	void applyGravity(const Vec3& force);
	void applyExternalForce();
	void dampingForce();
	void updatePosition(const float& timeStep);
	void updateVelocity(const float& timeStep);

	void collisionResolve(const float& deltaTime);

	void collisionPLane(
		const float& deltaTime,
		const Vec3& n,
		const float offset,
		const float friction);

	void collisionRigidBodies(const float& deltaTime);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif