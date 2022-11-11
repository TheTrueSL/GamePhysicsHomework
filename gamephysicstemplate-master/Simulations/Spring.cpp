#include "Spring.h"

Spring::Spring(float k, float d, float L, int p0, int p1)
{
	this->k = k;
	this->d = d;
	this->L = L;
	this->p0 = p0;
	this->p1 = p1;
}

Spring::~Spring()
{
}

void Spring::setSpringConstant(float newK)
{
	k = newK;
}

void Spring::setDampingConstant(float newD)
{
	d = newD;
}

void Spring::setRestLength(float newL)
{
	L = newL;
}

int Spring::getFirstPointInd()
{
	return p0;
}

int Spring::getSecondPointInd()
{
	return p1;
}
