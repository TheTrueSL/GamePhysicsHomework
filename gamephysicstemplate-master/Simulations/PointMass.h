#include "Simulator.h"
#ifndef POINTMASS_H
#define POINTMASS_H
class PointMass
{
public:
	//Constructors
	PointMass(float mass, Vec3 position, Vec3 Velocity, bool isFixed);

	//Destructors
	~PointMass();

	//Set Methods
	void setPointPosition(Vec3 newPosition);
	void setPointVelocity(Vec3 newVelocity);
	void setPointMass(float newMass);
	void setFixedPosition(bool newFixed);

	//Getters
	Vec3 getPointPosition();
	Vec3 getPointVelocity();

private:
	//Data Attributes
	float pointMass;
	Vec3 pointPosition;
	Vec3 pointVelocity;
	bool isPointFixed;

};
#endif

