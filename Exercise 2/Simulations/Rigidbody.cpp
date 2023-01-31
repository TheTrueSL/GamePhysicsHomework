#include "Rigidbody.h"

using namespace GamePhysics;

int Rigidbody::id_count = 0;
std::map<int, Rigidbody*> Rigidbody::dict = std::map<int, Rigidbody*>();

Rigidbody::Rigidbody(Transform* transform)
{
	// append global dictionary
	this->id = Rigidbody::id_count++;
	Rigidbody::dict[this->id] = this;
	//
	mass = 1;
	inverseMass = 1;
	isFixed = false;
	fixRotation = false;
	this->friction = 1.0f;

	this->transform = transform;

	init();

	inertia.setIdentity();
	inverseInertia.setIdentity();
}

Rigidbody::~Rigidbody()
{
	Rigidbody::dict.erase(id);
}

void GamePhysics::Rigidbody::setSphereInertia(float m, float r)
{
	CoM = Vec3(0,0,0);

	this->mass = m;
	this->inverseMass = 1.0 / m;

	float I = m * r * r * 2.0 / 5.0;

	inertia.setZero();
	inertia.value[0][0] = I;
	inertia.value[1][1] = I;
	inertia.value[2][2] = I;

	inverseInertia.setZero();
	inverseInertia.value[0][0] = 1.0 / I;
	inverseInertia.value[1][1] = 1.0 / I;
	inverseInertia.value[2][2] = 1.0 / I;
}

void GamePhysics::Rigidbody::setBoxInertia(float m, Vec3 size)
{
	CoM = Vec3(0,0,0);
	this->mass = m;
	this->inverseMass = 1.0 / m;

	float Ixx = m * (size.y * size.y + size.z * size.z) / 12.0;
	float Iyy = m * (size.x * size.x + size.z * size.z) / 12.0;
	float Izz = m * (size.x * size.x + size.y * size.y) / 12.0;
	
	inertia.setZero();
	inertia.value[0][0] = Ixx;
	inertia.value[1][1] = Iyy;
	inertia.value[2][2] = Izz;

	inverseInertia = inertia.inverse();
}

void GamePhysics::Rigidbody::addForce(Vec3 wforce, Vec3 wpoint)
{
	this->force += wforce;
	this->torque += cross(wpoint - worldCoM, wforce);
}

void GamePhysics::Rigidbody::update()
{
	Mat3 rotateMat(transform->rotation);
	worldCoM = transform->transformation.transformVector(CoM);
	worldInvInertia = rotateMat * inverseInertia * rotateMat.transpose();
}

void GamePhysics::Rigidbody::init()
{
	this->velocity = Vec3();
	this->angularMomentum = Vec3();
	this->angularVelocity = Vec3();
	this->virtualVelocity = Vec3();

	this->force = Vec3();
	this->torque = Vec3();
}
