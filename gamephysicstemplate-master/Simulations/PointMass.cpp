#include "PointMass.h"

PointMass::PointMass(float mass, Vec3 position, Vec3 Velocity, bool isFixed)
{
	pointMass = mass;
	pointPosition = position;
	pointVelocity = Velocity;
	isPointFixed = isFixed;
}

PointMass::~PointMass()
{
}

void PointMass::setPointPosition(Vec3 newPosition)
{
	pointPosition = newPosition;
}

void PointMass::setPointMass(float newMass)
{
	pointMass = newMass;
}

void PointMass::setFixedPosition(bool newFixed)
{
	isPointFixed = newFixed;
}

Vec3 PointMass::getPointPosition()
{
	return pointPosition;
}

Vec3 PointMass::getPointVelocity()
{
	return pointVelocity;
}

void PointMass::setPointVelocity(Vec3 newVelocity)
{
	pointVelocity = newVelocity;
}




