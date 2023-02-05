#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h

#include "Simulator.h"

#include "GameManager.h"

//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
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

	// events
	void onMouseDrag(int x, int y);
	void onMousePressed(int x);
	void onMouseReleased(int x);

	void onKeyboardPressed(UINT key);
	void onKeyboardReleased(UINT key);

	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Scene Creation
	void loadTestSetup();

public:

	void bindCustomObject(CustomObject*);
	void bindGameObject(GameObject*);
	void bindSpring(Spring*);

private:
	// mouse state for click detection
	bool _isPressed = false;
	time_t _pressedTimer;

	void onMouseDown(int x, int y);
	void onMouseDouble(int x, int y);

	Collider* dragCollider;
	void screen2Ray(int x, int y, Vec3& outO, Vec3& outD);

	Vec3 _gravity;
	float _linearDamping;
	float _angularDamping;

	bool _pause;

private:
	// Simulation
	std::vector<GameObject*> objects;
	std::vector<GameObject*> fixedObjects;
	std::vector<CustomObject*> customObjects;
	std::vector<CustomObject*> customFixedObjects;
	std::vector<Spring*> springs;
	GameManager* gameManager;
	std::map<int, CustomObject*> colliderMap;
	Character player;

	float _passedTime;

	void deleteObjects();

	void onStepStart(const float dt);
	void onStepEnd(const float dt);
	void integrateExplicitEuler(const float& timeStep);
	void clearForce();
	void computeSpringForce();
	void applyGravity(const Vec3& force);
	void applyDamping(const float& timeStep);
	void updatePosition(const float& timeStep);
	void updateVelocity(const float& timeStep);

	void collisionResolve(const float& deltaTime);

	void collisionPLane(
		const float& deltaTime,
		const Vec3& n,
		const float offset,
		const float friction);

	void collisionRigidBodies(const float& deltaTime);

	void updateTransformations();

private:
	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	};
#endif
