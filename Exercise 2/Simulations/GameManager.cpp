#include "GameManager.h"

#include "RigidBodySystemSimulator.h"

using namespace GamePhysics;

#define KEY_SPACE 32
#define KEY_UP 38
#define KEY_DOWN 40
#define KEY_LEFT 37
#define KEY_RIGHT 39

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
	level = 0;
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
		transform->position = Vec3(0, 0.6, 6.8);
		transform->rotation = Quat(0, 0, 0, 1);
		transform->rotation /= transform->rotation.norm();
		Rigidbody* rigidbody = new Rigidbody(transform);
		
		Collider* collider = new Collider(transform, rigidbody);	
		collider->layer = 0b100;
		collider->filter = 0b0100;
		collider->isTrigger = true;
		rigidbody->isFixed = true;
		collider->setBox(Vec3(6.4, 1.8, 0.3));
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
		int n = 8;
		float size = 15;
		float w = size / n;
		float ox = 4.9;
		float oy = 0.23;
		float oz = 8.0;
		for (int i = 0; i < n; i++) {
			Transform* t = new Transform();
			t->position = Vec3(ox + i * w * 1.01, oy, oz);
			t->rotation = Quat(0, 0, 0, 1);
			t->rotation /= t->rotation.norm();
			Rigidbody* rb = new Rigidbody(t);
			rb->setBoxInertia(1, Vec3(w, 0.6, 0.05));
			Collider* col = new Collider(t, rb);
			col->setBox(Vec3(w, 0.6, 0.05));
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
		}
		for (int i = 0; i < n; i++) {
			Transform* t = new Transform();
			t->position = Vec3(-ox - i * w * 1.01, oy, oz);
			t->rotation = Quat(0, 0, 0, 1);
			t->rotation /= t->rotation.norm();
			Rigidbody* rb = new Rigidbody(t);
			rb->setBoxInertia(1, Vec3(w, 0.6, 0.05));
			Collider* col = new Collider(t, rb);
			col->setBox(Vec3(w, 0.6, 0.05));
			GameObject* newPoint = new GameObject(t, rb, col);
			simulator->bindGameObject(newPoint);
		}
	}

	if (level > 0) {
		int n = 24;
		float w = 0.25;
		float h = 0.85;
		float rx = 8.0;
		float ry = 0.25;
		float rz = 6.0;
		for (int i = 0; i < n; i++) {
			Transform* bt = new Transform();
			float x = (rand() % 1000) * 0.001;
			float y = (rand() % 1000) * 0.001;
			float z = (rand() % 1000) * 0.001;
			x = (2 * x - 1) * rx;
			y = y * ry + h * 0.4;
			z = z * rz;
			bt->position = Vec3(x, y, z);

			Rigidbody* brb = new Rigidbody(bt);
			brb->setBoxInertia(0.6, Vec3(w, h, w));
			Collider* bcol = new Collider(bt, brb);
			bcol->setBox(Vec3(w, h, w));
			GameObject* body = new GameObject(bt, brb, bcol);
			simulator->bindGameObject(body);

			Transform* ht = new Transform();
			ht->position = Vec3(x, y + 0.51 * h, z);
			Rigidbody* hrb = new Rigidbody(ht);
			hrb->setSphereInertia(0.2, w * 0.5);
			Collider* hcol = new Collider(ht, hrb);
			hcol->setSphere(w * 0.5);
			GameObject* head = new GameObject(ht, hrb, hcol);
			simulator->bindGameObject(head);
			simulator->bindSpring(new Spring(hrb, brb, 5, 0.01, 0, Vec3(0, -w * 0.5, 0), Vec3(0, h * 0.5, 0)));
		}
	}

	if (level > 1) {
		int n = 4;
		int m = 10;
		float w = 0.5;
		
		for (int c = 0; c < m; c++) {

			float x = (c - m * 0.5) * w * 1.2;
			float y = n * w;
			float z = 3;
			Rigidbody* lastBody = nullptr;
			{
				Transform* bt = new Transform();
				bt->position = Vec3(x, y, z);
				Rigidbody* brb = new Rigidbody(bt);
				brb->setBoxInertia(0.05, Vec3(w, w, 0.05));
				brb->isFixed = true;
				Collider* bcol = new Collider(bt, brb);
				bcol->setBox(Vec3(w, w, 0.05));
				GameObject* body = new GameObject(bt, brb, bcol);
				simulator->bindGameObject(body);

				lastBody = brb;
			}
			for (int i = 1; i < n; i++) {
				Transform* bt = new Transform();
				bt->position = Vec3(x, y - i * w, z);
				Rigidbody* brb = new Rigidbody(bt);
				brb->setBoxInertia(0.05, Vec3(w, w, 0.05));
				Collider* bcol = new Collider(bt, brb);
				bcol->setBox(Vec3(w, w, 0.05));
				GameObject* body = new GameObject(bt, brb, bcol);
				simulator->bindGameObject(body);
				Spring* s0 = new Spring(lastBody, brb, 12, 0.001, 0.1, Vec3(w * 0.5, -w * 0.5, 0), Vec3(w * 0.5, w * 0.5, 0));
				Spring* s1 = new Spring(lastBody, brb, 12, 0.001, 0.1, Vec3(-w * 0.5, -w * 0.5, 0), Vec3(-w * 0.5, w * 0.5, 0));
				s0->tolerance = 2;
				s1->tolerance = 2;
				simulator->bindSpring(s0);
				simulator->bindSpring(s1);
				lastBody = brb;
			}
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
		if (!is_gameover && passed_time > 3 && (ball->rigidbody->velocity.z < 5e-3 || ball->transform->position.z > 8 || abs(ball->transform->position.x) > 8)) {
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
		onRestart();
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

void GameManager::onRestart()
{
	ball->rigidbody->init();
	ball->transform->position = Vec3();
	ball->transform->rotation = Quat(0, 0, 0, 1);
	player->init();
	if (level > 0) {

	}

	if (level > 1) {

	}
}
