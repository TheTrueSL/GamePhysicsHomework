#include "Character.h"
#include "Ball.h"
#include "DrawingUtilitiesClass.h"

#define KEY_SPACE 32
#define KEY_UP 38
#define KEY_DOWN 40
#define KEY_LEFT 37
#define KEY_RIGHT 39

using namespace GamePhysics;

const float ds = 5;

GamePhysics::Character::Character()
{
	dragZoffset = 0;
	buildModel();
}

GamePhysics::Character::~Character()
{
}

void GamePhysics::Character::onUpdate(const float dt)
{
	if (pressed[0]) {
		keeper.rigidbody->velocity.y = -ds;
		//keeper.rigidbody->force.y = 0;
	}
	if (pressed[1]) {
		keeper.rigidbody->velocity.y = ds;
		//keeper.rigidbody->force.y = 0;
	}
	if (pressed[2]) {
		keeper.rigidbody->velocity.x = -ds;
		//keeper.rigidbody->force.x = 0;
	}
	if (pressed[3]) {
		keeper.rigidbody->velocity.x = ds;
		//keeper.rigidbody->force.x = 0;
	}

	keeper.transform->position.z = ancher[0].transform->position.z;
	keeper.rigidbody->velocity.z = 0;
	keeper.transform->position.x = std::min(ancher[2].transform->position.x,
		std::max(ancher[1].transform->position.x, keeper.transform->position.x));
	keeper.transform->position.y = std::min(ancher[1].transform->position.y,
		std::max(ancher[0].transform->position.y, keeper.transform->position.y));
	keeper.rigidbody->angularMomentum -= keeper.rigidbody->angularMomentum * dt;
	keeper.rigidbody->angularVelocity -= keeper.rigidbody->angularVelocity * dt;

}

void GamePhysics::Character::draw(DrawingUtilitiesClass* duc)
{
	Vec3 dir(0.01, -2.0, 0.01);
	float y_plane = -0.5;
	normalize(dir);
	duc->setUpLighting(Vec3(), Vec3(1, 1, 1), 10, Vec3(1,0,0));
	keeper.draw(duc);
}

void GamePhysics::Character::init(Vec3 positon)
{
	for (int i = 0; i < 4; i++)
	{
		pressed[i] = false;
	}
	keeper.rigidbody->init();
}

void GamePhysics::Character::updateTransformations()
{
	keeper.update();
}

void GamePhysics::Character::onKeyPressed(unsigned int key)
{
	if (key == KEY_DOWN) {
		pressed[0] = true;
	}
	if (key == KEY_UP) {
		pressed[1] = true;
	}
	if (key == KEY_LEFT) {
		pressed[2] = true;
	}
	if (key == KEY_RIGHT) {
		pressed[3] = true;
	}
	if (key == KEY_SPACE) {
		pressed[4] = true;
	}
}

void GamePhysics::Character::onKeyReleased(unsigned int key)
{
	if (key == KEY_DOWN) {
		pressed[0] = false;
		keeper.rigidbody->velocity.y = 0;
	}
	if (key == KEY_UP) {
		pressed[1] = false;
		keeper.rigidbody->velocity.y = 0;
	}
	if (key == KEY_LEFT) {
		pressed[2] = false;
		keeper.rigidbody->velocity.x = 0;
	}
	if (key == KEY_RIGHT) {
		pressed[3] = false;
		keeper.rigidbody->velocity.x = 0;
	}
	if (key == KEY_SPACE) {
	}
}

void GamePhysics::Character::onMouseMove(int x, int y, Vec3 ro, Vec3 rd)
{
	float time_passed = -ticker;
	ticker = clock();
	time_passed = time_passed + ticker;
	time_passed *= 0.001;
	Vec3 p;
	Vec3 newPos;
	float z = dragZoffset;
	if (planeIntersection(Vec3(0,0,0), Vec3(0, 0.1, 1), ro, rd, p)) {
		z = z + 0.25 * p.y;
		newPos = Vec3(p.x, p.y, z);
	}
	else {
		newPos = Vec3(0, 0, z);
	}
	float lin_coef = 2;
	float rot_coef = 0.6;
	ball->transform->position = newPos;

	ball->rigidbody->velocity *= std::max(0.0f, 1 - time_passed);
	ball->rigidbody->angularMomentum *= std::max(0.0f, 1 - time_passed);
	Vec3 dPos = (newPos - lastPos);
	ball->rigidbody->velocity += Vec3(dPos.x, dPos.y, dPos.z + norm(dPos)) * lin_coef;
	ball->rigidbody->angularMomentum += cross(lastdPos, dPos) * rot_coef;
	lastPos = newPos;
	lastdPos = dPos;
	ball->update();
}

void GamePhysics::Character::onMousePressed(int x, int y, Vec3 ro, Vec3 rd)
{
	ticker = clock();
	Vec3 p;
	float z = dragZoffset;
	if (planeIntersection(Vec3(), Vec3(0, 1, -1), ro, rd, p)) {
		z = z + p.y * 0.25;
		lastPos = Vec3(p.x, p.y, z);
	}
	else {
		lastPos = Vec3(0, 0, z);
	}
}

void GamePhysics::Character::onMouseReleased(int x, int y)
{
	
}

void GamePhysics::Character::attachBall(Ball* b) 
{
	this->ball = b;
}

void GamePhysics::Character::buildModel()
{
	{
		Transform* transform = new Transform();
		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);

		transform->position = Vec3(0, 0, 6.1);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		rigidbody->mass = 4;
		rigidbody->isFixed = false;
		rigidbody->useGravity = false;

		collider->layer = 0b100;
		collider->filter = 0b100;
		collider->setBox(Vec3(0.75,0.75, 0.1));

		this->keeper.regist(transform, rigidbody, collider);
	}
	{
			Transform* transform = new Transform();
			Rigidbody* rigidbody = new Rigidbody(transform);
			Collider* collider = new Collider(transform, rigidbody);

			transform->position = Vec3(-3, -0.5, 6.1);
			transform->rotation = Quat(0, 0, 0, 1);
			transform->rotation /= transform->rotation.norm();
			rigidbody->isFixed = true;
			collider->layer = 0b100;
			collider->filter = 0b100;
			collider->setSphere(0.03);

			this->ancher[0].regist(transform, rigidbody, collider);
	}
	{
		Transform* transform = new Transform();
		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);

		transform->position = Vec3(-3, 2, 6.1);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();
		rigidbody->isFixed = true;
		collider->layer = 0b100;
		collider->filter = 0b100;
		collider->setSphere(0.03);

		this->ancher[1].regist(transform, rigidbody, collider);
	}
	{
		Transform* transform = new Transform();
		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);

		transform->position = Vec3(3, 2, 6.1);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();
		rigidbody->isFixed = true;
		collider->layer = 0b100;
		collider->filter = 0b100;
		collider->setSphere(0.03);

		this->ancher[2].regist(transform, rigidbody, collider);
	}
	{
		Transform* transform = new Transform();
		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);

		transform->position = Vec3(3, -0.5, 6.1);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();
		rigidbody->isFixed = true;
		collider->layer = 0b100;
		collider->filter = 0b100;
		collider->setSphere(0.03);

		this->ancher[3].regist(transform, rigidbody, collider);
	}
	{
		const float s = 10;
		const float d = 0.5;
		const float l = 0.001;
		this->springs[0] = new Spring(keeper.rigidbody.get(), ancher[0].rigidbody.get(), s, d, l,
			Vec3(-0.5, -0.5, 0), Vec3());
		this->springs[1] = new Spring(keeper.rigidbody.get(), ancher[1].rigidbody.get(), s, d, l,
			Vec3(-0.5, 0.5, 0), Vec3());
		this->springs[2] = new Spring(keeper.rigidbody.get(), ancher[2].rigidbody.get(), s, d, l,
			Vec3(0.5, 0.5, 0), Vec3());
		this->springs[3] = new Spring(keeper.rigidbody.get(), ancher[3].rigidbody.get(), s, d, l,
			Vec3(0.5, -0.5, 0), Vec3());
	}
	for (int i = 0; i < 4; i++) {
		ancher[i].update();
	}
}


bool GamePhysics::Character::planeIntersection(const Vec3& o, const Vec3& n, const Vec3& ro, const Vec3& rd, Vec3& p)
{
	float t = -1;
	float denom = dot(n, rd);
	if (abs(denom) > 1e-6) {
		Vec3 v = o - ro;
		t = dot(v, n) / denom;

		p = ro + t * rd;
	}

	return t >= 0;
}
