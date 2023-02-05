#include "Character.h"

#define KEY_UP 38
#define KEY_DOWN 40
#define KEY_LEFT 37
#define KEY_RIGHT 39

using namespace GamePhysics;

GamePhysics::Character::Character()
{
	buildModel();
}

GamePhysics::Character::~Character()
{
}

void GamePhysics::Character::onUpdate(const float dt)
{
}

void GamePhysics::Character::draw(DrawingUtilitiesClass* duc)
{
	Vec3 dir(0.01, -2.0, 0.01);
	float y_plane = -0.5;
	normalize(dir);

}

void GamePhysics::Character::init(Vec3 positon)
{
}

void GamePhysics::Character::updateTransformations()
{
}

void GamePhysics::Character::onKeyPressed(unsigned int)
{
}

void GamePhysics::Character::onKeyReleased(unsigned int)
{
}

void GamePhysics::Character::onMouseMove(int, int)
{
}

void GamePhysics::Character::onMousePressed(int)
{
}

void GamePhysics::Character::buildModel()
{
}
