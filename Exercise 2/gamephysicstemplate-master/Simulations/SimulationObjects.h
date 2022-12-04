#pragma once
#include "Simulator.h"

class Mat3 {
public:
	float value[3][3];
	// Constructor
	inline Mat3(void);
	// Copy-Constructor
	inline Mat3(const Mat3& m);
	Mat3(const Mat4& m);
	Mat3(const Quat& q);
	// Assignment operator
	inline const Mat3& operator=  (const Mat3& m);
	// Assign and mult operator
	inline const Mat3& operator*= (float s);
	// Assign and div operator
	inline const Mat3& operator/= (float s);

	// unary operator
	inline Mat3 operator- () const;

	// binary operator add
	inline Mat3 operator+ (const Mat3&) const;
	// binary operator sub
	inline Mat3 operator- (const Mat3&) const;
	// binary operator mult
	inline Mat3 operator* (const Mat3&) const;
	// binary operator mult
	Vec3 dot(const Vec3&) const;
	Vec3 dotLefthand(const Vec3&) const;
	// binary operator mult
	inline Mat3 operator* (float) const;
	// binary operator div
	inline Mat3 operator/ (float) const;

	inline void setZero();
	inline void setIdentity();
	inline Mat3 inverse();
	inline Mat3 transpose();
};

class PointMass {
	friend class RigidBodySystemSimulator;
	friend class RigidBody;
public:
	PointMass();
	PointMass(float mass, const Vec3& position, const Vec3& velocity, float radius, bool isFixed, 
		bool _isBullet=false);

private:
	bool _isFixed;
	bool _isBullet;

	Vec3 _lastPosition;
	Vec3 _position;

	Vec3 _velocity;
	Vec3 _force;

	float _radius;
	float _mass;
	float _invMass;
	Vec3 _acceleration;
};

class Spring {
	friend class RigidBodySystemSimulator;
public:
	Spring(int pID0, int pID1, float stiffness, float damping, float restLength);
private:
	int _pointID0;
	int _pointID1;

	float _damping;
	float _stiffness;
	float _restLength;
};

class Contact {
public:
	Vec3 contactPoint;
	Vec3 contactNormal;
	float contactDistance;
};

class RigidBody {
	friend class RigidBodySystemSimulator;
public:
	RigidBody(float mass, const Vec3& size, const Vec3& position, bool fixed, float friction=25);
	const Mat4& getTransformation();
	void addExternalForce(const Vec3& worldForce, const Vec3& worldPos);
	void updateTransofmration();
	void checkSleep();
	void wake();
	bool rayIntersection(const Vec3& rayOrigin, const Vec3& rayDir, Vec3& outIntersection);
	bool isOverlapCoarse(const Vec3& point, const float& radius);
	bool pointContact(const Vec3& pos, const float radius, Contact& outContact);
	bool planeContact(const Vec3& normal, const float& offset, std::vector<Contact>& outContacts);
	bool checkSATContact(RigidBody& other, Contact& outContact);

public:
	// sat informations
	struct Projection {
		float max;
		float min;
	};
	struct Edge {
		int idx0;
		int idx1;
	};

protected:
	void boxInertia(const Vec3& size);
private:
	Quat _orientation;
	Vec3 _position;
	Vec3 _lastPosition;
	Vec3 _velocity; // world space
	Vec3 _angularVelocity;
	Vec3 _angularMomentum;

	Mat4 _renderTransformation;
	Mat4 _transformation;
	Mat4 _invTransformation;

	Vec3 _force;
	Vec3 _torque; // local space

	Vec3 _size;
	Vec3 _halfSize;
	Mat3 _inertia;
	Mat3 _invInertia;

	Vec3 _coarseCenter;
	float _coarseRadius;
	vector<Vec3> _corners;
	vector<Vec3> _worldCorners;

	Vec3 _com;
	float _mass;
	float _invMass;
	float _friction;

	bool _isFixed;
	bool _isSleep;
	int _restTimer = 0;

	static int RestFrames;
	static float RestThreshold;

	Mat3 _worldInvInertia;
	Vec3 _worldCom;

	Vec3 _basisAxis[3];
};
