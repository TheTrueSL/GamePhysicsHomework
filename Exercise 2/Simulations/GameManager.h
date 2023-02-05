#pragma once

#include "GameObject.h"
#include "Spring.h"
#include "Character.h"
#include "Ball.h"
#include "Goal.h"

class RigidBodySystemSimulator;

class GameManager
{
public:
	GameManager(RigidBodySystemSimulator*, GamePhysics::Character*);
	~GameManager();
	
	void onStart();

	void onFrameUpdate();
	

private:
	RigidBodySystemSimulator* simulator;
	GamePhysics::Character* player;
	GamePhysics::Ball* ball;
	GamePhysics::Goal* goal;
	
	std::vector<GamePhysics::Rigidbody*> netGrid;
};
