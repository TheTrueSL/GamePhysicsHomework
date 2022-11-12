#include "Simulator.h"
#ifndef SPRING_H
#define SPRING_H
class Spring
{
public:
	//Constructors
	Spring(float k, float d, float L, int p0, int p1);

	//Destructors
	~Spring();

	//Functions
	void setSpringConstant(float newK);
	void setDampingConstant(float newD);
	void setRestLength(float newL);

	int getFirstPointInd();
	int getSecondPointInd();
	float getRestLength();

private:
	float k;
	float d;
	float L;
	int p0;
	int p1;
};
#endif


