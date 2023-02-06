#include "GameManager.h"

#include "RigidBodySystemSimulator.h"

using namespace GamePhysics;

void HSVtoRGB(float H, float S, float V, float& R, float& G, float& B) {
	//H(Hue): 0-360 degrees
	//S(Saturation) : 0 - 100 percent
	//V(Value) : 0 - 100 percent

	R = 0;
	G = 0;
	B = 0;

	if (H > 360 || H < 0 || S>100 || S < 0 || V>100 || V < 0) {
		cout << "The givem HSV values are not in valid range" << endl;
		return;
	}

	float s = S * 0.01;
	float v = V * 0.01;
	float C = s * v;
	float X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	float m = v - C;
	float r, g, b;
	if (H >= 0 && H < 60) {
		r = C, g = X, b = 0;
	}
	else if (H >= 60 && H < 120) {
		r = X, g = C, b = 0;
	}
	else if (H >= 120 && H < 180) {
		r = 0, g = C, b = X;
	}
	else if (H >= 180 && H < 240) {
		r = 0, g = X, b = C;
	}
	else if (H >= 240 && H < 300) {
		r = X, g = 0, b = C;
	}
	else {
		r = C, g = 0, b = X;
	}

	R = (r + m);
	G = (g + m);
	B = (b + m);
}


GameManager::GameManager(RigidBodySystemSimulator* simulator, Character* character)
{
	this->simulator = simulator;
	this->player = character;
}

GameManager::~GameManager()
{
}

void GameManager::onStart()
{
	is_gameover = false;
	effectTimer = 0;
	passed_time = 0;
	player->init();
	// ball 0
	{
		Transform* transform = new Transform();
		//transform->position = Vec3(0.5, -0.4, 0);
		transform->position = Vec3(0, 0, 0);
		transform->rotation = Quat(0, 0, 0, 1);

		Rigidbody* rigidbody = new Rigidbody(transform);
		rigidbody->friction = 0.5;
		/*rigidbody->velocity = Vec3(0.1, 0.1, 4);
		rigidbody->angularMomentum = Vec3(0.1, 0.1, 0.0);*/
		rigidbody->setSphereInertia(0.25, 0.1);

		Collider* collider = new Collider(transform, rigidbody);
		collider->drawAxis = true;
		collider->layer = 0b100;
		collider->ballFlag = true;
		collider->setSphere(0.2);

		ball = new Ball();
		ball->regist(transform, rigidbody, collider);
		// binded objects will be automatically deleted by simulator when reset() is called;
		simulator->bindCustomObject(ball);
		player->attachBall(ball);
	}
	//Goal Detection Box Setup
	{
		Transform* transform = new Transform();
		transform->position = Vec3(0, 0.6, 6.5);
		transform->rotation = Quat(0, 0, 1, 1);
		transform->rotation /= transform->rotation.norm();
		Rigidbody* rigidbody = new Rigidbody(transform);
		
		Collider* collider = new Collider(transform, rigidbody);	
		collider->layer = 0b100;
		collider->filter = 0b0100;
		collider->isTrigger = true;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(2.5, 7.5, 0.2));
		goal = new Goal(this);
		goal->regist(transform, rigidbody, collider);
		simulator->bindCustomObject(goal);
	}
	//Create Goal
	{
		Transform* transform = new Transform();
		transform->position = Vec3(0, 1.75, 6.5);
		transform->rotation = Quat(0, 0, 1, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		rigidbody->isFixed = true;
		collider->layer = 0b100;
		collider->filter = 0b100;
		collider->setBox(Vec3(0.2, 7.5, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Top Back Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(0, 1.75, 8);
		transform->rotation = Quat(0, 0, 1, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		collider->layer = 0b100;
		collider->filter = 0b100;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 7.5, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Right Front Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(-3.75, 0.6, 6.5);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		collider->layer = 0b100;
		collider->filter = 0b100;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Right Back Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(-3.75, 0.6, 8);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		collider->layer = 0b100;
		collider->filter = 0b100;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Left Front Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(3.75, 0.6, 6.5);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		collider->layer = 0b100;
		collider->filter = 0b100;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Left Back Post
	{
		Transform* transform = new Transform();
		transform->position = Vec3(3.75, 0.6, 8);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();

		Rigidbody* rigidbody = new Rigidbody(transform);
		Collider* collider = new Collider(transform, rigidbody);
		collider->layer = 0b100;
		collider->filter = 0b100;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(0.2, 2.25, 0.2));

		simulator->bindGameObject(new GameObject(transform, rigidbody, collider));
	}
	//Top Surface
	{
		Transform* t = new Transform();
		t->position = Vec3(0, 1.75, 7.25);
		t->rotation = Quat(1, 0, 0, 1);
		t->rotation /= t->rotation.norm();
		Rigidbody* rb = new Rigidbody(t);
		rb->isFixed = true;
		Collider* col = new Collider(t, rb);
		col->layer = 0b100;
		col->filter = 0b100;
		col->setBox(Vec3(7.5, 1.25, 0.01));
		GameObject* newPoint = new GameObject(t, rb, col);
		simulator->bindGameObject(newPoint);
	}
	//Left Surface
	{
		Transform* t = new Transform();
		t->position = Vec3(-3.75, 0.6, 7.25);
		t->rotation = Quat(0, 1, 0, 1);
		t->rotation /= t->rotation.norm();
		Rigidbody* rb = new Rigidbody(t);
		rb->isFixed = true;
		Collider* col = new Collider(t, rb);
		col->layer = 0b100;
		col->filter = 0b100;
		col->setBox(Vec3(1.7, 2.25, 0.01));
		GameObject* newPoint = new GameObject(t, rb, col);
		simulator->bindGameObject(newPoint);
	}
	//Right Surface
	{
		Transform* t = new Transform();
		t->position = Vec3(3.75, 0.6, 7.25);
		t->rotation = Quat(0, 1, 0, 1);
		t->rotation /= t->rotation.norm();
		Rigidbody* rb = new Rigidbody(t);
		rb->isFixed = true;
		Collider* col = new Collider(t, rb);
		col->layer = 0b100;
		col->filter = 0b100;
		col->setBox(Vec3(1.7, 2.25, 0.01));
		GameObject* newPoint = new GameObject(t, rb, col);
		simulator->bindGameObject(newPoint);
	}

	{
		//Backside of the grid
		Rigidbody* oldPoint = NULL;
		float stiffness = 1.0f;
		float dampening = 1.0f;
		float  length = 0.21f;
		float mass = 0.02f;

		for (int i = 0; i < 13; i++) {
			for (int j = 0; j < 38; j++) {
				Transform* t = new Transform();
				t->position = Vec3(-3.75 + (0.2 * j), 1.75 - (0.2 * i), 8);
				Rigidbody* rb = new Rigidbody(t);
				//Change here to false to activate physics
				if (i == 0 || i == 12 || j == 0 || j == 37) {
					rb->isFixed = true;
				}
				else {
					rb->isFixed = false;
				}
				rb->useGravity = false;
				rb->mass = mass;
				Collider* col = new Collider(t, rb);
				col->layer = 0b1000;
				col->filter = 0b0110;
				col->setSphere(0.001);
				GameObject* newPoint = new GameObject(t, rb, col);
				netGrid.push_back(rb);
				simulator->bindGameObject(newPoint);
				if (!j == 0) {
					simulator->bindSpring(new Spring(oldPoint, rb, stiffness, dampening, length, Vec3(0, 0, 0), Vec3(0, 0, 0)));
				}
				oldPoint = rb;
			}
		}

		for (int z=38; z<netGrid.size()-1; z++)
		{
			Rigidbody* rb1 = netGrid[z];
			Rigidbody* rb2 = netGrid[z-38];
			simulator->bindSpring(new Spring(rb1, rb2, stiffness, dampening, length, Vec3(0, 0, 0), Vec3(0, 0, 0)));
		}

	}
}

void GameManager::onFrameUpdate(DrawingUtilitiesClass* duc, ID3D11DeviceContext* ctx)
{
	if (is_gameover && effectTimer > 0) {
		const float T = 0.2;
		float t = round((sinf(effectTimer / T) + 1) * 0.5f);
		if (hit_goal) {
			float h0 = 100;
			float s0 = 100;
			float h1 = 40;
			float s1 = 90;
			float r, g, b;
			HSVtoRGB(h0 * t + h1 * (1 - t), s0 * t + s1 * (1-t), 100, r, g, b);
			Vec3 c0(r, g, b);
			t = 1 - t;
			HSVtoRGB(h0 * t + h1 * (1 - t), s0 * t + s1 * (1 - t), 100, r, g, b);
			Vec3 c1(r, g, b);
			duc->DrawFloor(ctx, c0, c1);
		}
		else {
			float h0 = 0;
			float s0 = 100;
			float h1 = 60;
			float s1 = 90;
			float r, g, b;
			HSVtoRGB(h0 * t + h1 * (1 - t), s0 * t + s1 * (1 - t), 100, r, g, b);
			Vec3 c0(r, g, b);
			t = 1 - t;
			HSVtoRGB(h0 * t + h1 * (1 - t), s0 * t + s1 * (1 - t), 100, r, g, b);
			Vec3 c1(r, g, b);
			duc->DrawFloor(ctx, c0, c1);
		}
	}
	else {
		duc->DrawFloor(ctx, Vec3 (0.26, 0.52, 0.0), Vec3(0.24f, 0.48f, 0.0f));
	}
}

void GameManager::onPhysicUpdate(float dt)
{
	if (is_gameover && effectTimer > 0) {
		effectTimer -= dt;
	}
	if (!player->chance) {
		passed_time += dt;
		if (!is_gameover && passed_time > 5 && (ball->rigidbody->velocity.z < 1e-3 || norm(ball->transform->position) > 8)) {
			onGameOver(false);
		}
	}
}

void GameManager::onKeyPressed(unsigned int key)
{
	;
	if (is_gameover && key == 'R') {
		is_gameover = false;
		goal->scored = false;
		effectTimer = 0;
		passed_time = 0;
		ball->rigidbody->init();
		ball->transform->position = Vec3();
		ball->transform->rotation = Quat(0, 0, 0, 1);
		player->init();
	}
	else {
		player->onKeyPressed(key);
	}
}

void GameManager::onKeyReleased(unsigned int key)
{
	player->onKeyReleased(key);
}

void GameManager::onGameOver(bool hit_goal)
{
	if (!is_gameover) {
		is_gameover = true;
		effectTimer = 15;
		this->hit_goal = hit_goal;
	}
}
