#pragma once

#include <d3d11.h>

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

	void onFrameUpdate(DrawingUtilitiesClass* duc, ID3D11DeviceContext* ctx);
	void onPhysicUpdate(float dt);
	void onKeyPressed(unsigned int key);
	void onKeyReleased(unsigned int key);
	void onGameOver(bool hit_goal);

private:
	float effectTimer;
	float passed_time;
	bool hit_goal;
	bool is_gameover;
	RigidBodySystemSimulator* simulator;

	GamePhysics::Character* player;
	GamePhysics::Ball* ball;
	GamePhysics::Goal* goal;
	std::vector<GamePhysics::Rigidbody*> netGrid;
};
