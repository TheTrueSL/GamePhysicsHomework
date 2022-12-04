#include "SimulationObjects.h"

namespace {
	const Vec3 ZERO_VEC(0, 0, 0);

	const Vec3 BOX_CORNERS[8] = {
	Vec3(1,1,1),//0
	Vec3(1,1,-1),//1
	Vec3(1,-1,1),//2
	Vec3(1,-1,-1),//3
	Vec3(-1,1,1),//4
	Vec3(-1,1,-1),//5
	Vec3(-1,-1,1),//6
	Vec3(-1,-1,-1)//7
	};

	const RigidBody::Edge BOX_EDGES[12] = {
		{0, 1},{0, 2},{0, 4},
		{1, 3},{1, 5},
		{2, 3},{2, 6},
		{3, 7},
		{4, 5},{4, 6},
		{5, 7},
		{6, 7},
	};
}

int RigidBody::RestFrames = 4;
float RigidBody::RestThreshold = 1e-6;

PointMass::PointMass():PointMass(0, Vec3(), Vec3(), 0, false, false){

}

PointMass::PointMass(float mass, const Vec3& position, const Vec3& velocity, float radius, bool isFixed, 
	bool isBullet)
{
	_mass = mass;
	_invMass = 1.0 / mass;

	_radius = radius;
	_position = position;
	_lastPosition = position;

	_velocity = velocity;

	_force = ZERO_VEC;
	_acceleration = ZERO_VEC;

	_isFixed = isFixed;
	_isBullet = isBullet;
}

Spring::Spring(int pID0, int pID1, float stiffness, float damping, float restLength)
{
	_pointID0 = pID0;
	_pointID1 = pID1;

	_damping = damping;
	_stiffness = stiffness;
	_restLength = restLength;
}

void RigidBody::updateTransofmration()
{
	Mat4 translation;
	translation.initTranslation(_position.x, _position.y, _position.z);
	Mat4 scale;
	scale.initScaling(_size.x, _size.y, _size.z);

	_transformation = _orientation.getRotMat() * translation;
	_invTransformation = _transformation.inverse();
	_renderTransformation = scale * _transformation;

	Mat3 rotateMat(_orientation);
	_worldCom = _transformation.transformVector(_com);
	_worldInvInertia = rotateMat * _invInertia * rotateMat.transpose();

	for (int i = 0; i < 8; i++) {
		_worldCorners[i] = _transformation.transformVector(_corners[i]);
	}

	_coarseCenter = _worldCom;

	_basisAxis[0] = _transformation.transformVectorNormal(Vec3(1, 0, 0));
	_basisAxis[1] = _transformation.transformVectorNormal(Vec3(0, 1, 0));
	_basisAxis[2] = _transformation.transformVectorNormal(Vec3(0, 0, 1));
}

bool RigidBody::isOverlapCoarse(const Vec3& point, const float& radius)
{
	float dist = normNoSqrt(point - _coarseCenter);
	return (dist < ((radius + _coarseRadius) * (radius + _coarseRadius)));
}

bool RigidBody::planeContact(const Vec3& normal, const float& offset, std::vector<Contact>& outContacts) {
	outContacts.clear();
	for (int i = 0; i < _worldCorners.size(); i++) {
		Vec3& corner = _worldCorners[i];
		float d = dot(corner, normal);
		//Vec3 vel = _velocity + cross(_angularVelocity, corner - _worldCom);
		if (d < offset) {
			Contact contact;
			contact.contactDistance = abs(offset - d);
			contact.contactNormal = normal;
			contact.contactPoint = corner;
			outContacts.push_back(contact);
		}
	}
	return (outContacts.size() > 0);
}

bool RigidBody::pointContact(const Vec3& pos, const float radius, Contact& outContact)
{
	const float epsilon = 1e-8;
	Vec3 localPoint = _invTransformation.transformVector(pos);
	float minDist = _halfSize.x - abs(localPoint.x) + radius;
	Vec3 normal = _basisAxis[0] * (localPoint.x < 0 ? -1 : 1);
	if (minDist < 0)
		return false;

	float dist = _halfSize.y - abs(localPoint.y) + radius;
	if (dist < 0)
		return false;
	else if (dist < minDist) {
		minDist = dist;
		normal = _basisAxis[1] * (localPoint.y < 0 ? -1 : 1);
	}

	dist = _halfSize.z - abs(localPoint.z) + radius;
	if (dist < 0)
		return false;
	else if (dist < minDist) {
		minDist = dist;
		normal = _basisAxis[2] * (localPoint.z < 0 ? -1 : 1);
	}

	outContact.contactNormal = -normal;
	if (minDist < radius) {
		outContact.contactDistance = minDist;
		outContact.contactPoint = pos - normal * radius;
	}
	else {
		outContact.contactDistance = minDist;
		outContact.contactPoint = pos;
	}
	
	return true;
}

bool RigidBody::rayIntersection(const Vec3& rayOrigin, const Vec3& rayDir, Vec3& outIntersection)
{
	Vec3 localRayOrigin = _invTransformation.transformVector(rayOrigin);
	Vec3 localRayDir = _invTransformation.transformVectorNormal(rayDir);
	Vec3 tMin = (-_halfSize - localRayOrigin) / localRayDir;
	Vec3 tMax = (_halfSize - localRayOrigin) / localRayDir;
	Vec3 t1(min(tMin.x, tMax.x), min(tMin.y, tMax.y), min(tMin.z, tMax.z));
	Vec3 t2(max(tMin.x, tMax.x), max(tMin.y, tMax.y), max(tMin.z, tMax.z));
	float tNear = max(max(t1.x, t1.y), t1.z);
	float tFar = min(min(t2.x, t2.y), t2.z);
	if (tNear > tFar)
		return false;

	outIntersection = _transformation.transformVector(localRayOrigin + tNear * localRayDir);

	return true;
}

RigidBody::RigidBody(float mass, const Vec3& size, const Vec3& position, bool fixed, float friction)
{
	_mass = mass;
	_invMass = 1.0 / mass;
	_friction = friction;

	_size = size;
	_halfSize = size * 0.5;
	_position = position;
	_lastPosition = _position;
	_orientation = Quat(0, 0, 0, 1);

	_velocity = ZERO_VEC;
	_angularVelocity = ZERO_VEC;
	_angularMomentum = ZERO_VEC;

	_force = ZERO_VEC;
	_torque = ZERO_VEC;

	_isFixed = fixed;
	_isSleep = false;

	for (int i = 0; i < 8; i++) {
		Vec3 corner = BOX_CORNERS[i] * _halfSize;
		_corners.push_back(corner);
	}
	_worldCorners = _corners;

	// calculate box inertia
	boxInertia(_size);
	updateTransofmration();

	_coarseRadius = 0;
	for (int i = 0; i < 8; i++) {
		float dist = norm(_corners[i] - _com);
		if (dist > _coarseRadius)
			_coarseRadius = dist;
	}
}

void RigidBody::wake() {
	_isSleep = false;
	_restTimer = 0;
}

void RigidBody::checkSleep()
{
	float velnorm = norm(_velocity) + norm(_angularVelocity);
	if (velnorm < RigidBody::RestThreshold) {
		_restTimer += 1;
		if (_restTimer > RigidBody::RestFrames) {
			_isSleep = true;
		}
	}
}

const Mat4& RigidBody::getTransformation()
{
	return _transformation;
}

void RigidBody::addExternalForce(const Vec3& worldForce, const Vec3& worldPos)
{
	//Vec3 localPos = _invTransformation.transformVectorNormal(worldPos);
	//Vec3 localForce = _invTransformation.transformVectorNormal(worldForce);
	_force += worldForce;
	_torque += cross(worldPos - _worldCom, worldForce);
}

void RigidBody::boxInertia(const Vec3& size)
{
	_com = ZERO_VEC;
	_inertia.setZero();
	// Tutorial ?????
	//// x component
	//float Cxx = (size.x * size.x * 0.25) * 8;
	//// y component
	//float Cyy = (size.y * size.y * 0.25) * 8;
	//// z component
	//float Czz = (size.z * size.z * 0.25) * 8;
	//float trace = Cxx + Cyy + Czz;
	//float Ixx = _mass * (trace - Cxx);
	//float Iyy = _mass * (trace - Cyy);
	//float Izz = _mass * (trace - Czz);
	// Wiki
	float Ixx = _mass * (size.y * size.y + size.z * size.z) / 12.0;
	float Iyy = _mass * (size.x * size.x + size.z * size.z) / 12.0;
	float Izz = _mass * (size.x * size.x + size.y * size.y) / 12.0;
	_inertia.value[0][0] = Ixx;
	_inertia.value[1][1] = Iyy;
	_inertia.value[2][2] = Izz;
	_invInertia = _inertia.inverse();
}

inline Mat3::Mat3(void)
{
	setZero();
}

Mat3::Mat3(const Quat& q)
{
	value[0][0] = 1 - 2 * (q.y * q.y + q.z * q.z);
	value[0][1] = 2 * (q.x * q.y - q.w * q.z);
	value[0][2] = 2 * (q.x * q.z + q.w * q.y);
	value[1][0] = 2 * (q.x * q.y + q.w * q.z);
	value[1][1] = 1 - 2 * (q.x * q.x + q.z * q.z);
	value[1][2] = 2 * (q.y * q.z - q.w * q.x);
	value[2][0] = 2 * (q.x * q.z - q.w * q.y);
	value[2][1] = 2 * (q.y * q.z + q.w * q.x);
	value[2][2] = 1 - 2 * (q.x * q.x + q.y * q.y);
}

inline Mat3::Mat3(const Mat3& m)
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			value[r][c] = m.value[r][c];
		}
	}
}

Mat3::Mat3(const Mat4& m)
{
	for (int c = 0; c < 3; c++) {
		value[0][c] = m.value[0][c];
		value[1][c] = m.value[1][c];
		value[2][c] = m.value[2][c];
	}
}

inline const Mat3& Mat3::operator=(const Mat3& m)
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			value[r][c] = m.value[r][c];
		}
	}
	return *this;
}

inline const Mat3& Mat3::operator*=(float s)
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			value[r][c] *= s;
		}
	}
	return *this;
}

inline const Mat3& Mat3::operator/=(float s)
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			value[r][c] /= s;
		}
	}
	return *this;
}

inline Mat3 Mat3::operator-() const
{
	Mat3 o;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			o.value[r][c] = -value[r][c];
		}
	}
	return o;
}

inline Mat3 Mat3::operator+(const Mat3& m) const
{
	Mat3 o;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			o.value[r][c] = value[r][c] + m.value[r][c];
		}
	}
	return o;
}

inline Mat3 Mat3::operator-(const Mat3& m) const
{
	Mat3 o;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			o.value[r][c] = value[r][c] - m.value[r][c];
		}
	}
	return o;
}

inline Mat3 Mat3::operator*(const Mat3& m) const
{
	Mat3 o;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			o.value[r][c] = 
				value[r][0] * m.value[0][c] + 
				value[r][1] * m.value[1][c] + 
				value[r][2] * m.value[2][c];
		}
	}
	return o;
}

Vec3 Mat3::dot(const Vec3& v) const
{
	Vec3 o;
	for (int r = 0; r < 3; r++) {
		o.value[r] =
			value[r][0] * v.value[0] +
			value[r][1] * v.value[1] +
			value[r][2] * v.value[2];
	}
	return o;
}

Vec3 Mat3::dotLefthand(const Vec3& v) const
{
	Vec3 o;
	for (int r = 0; r < 3; r++) {
			o.value[r] =
				value[0][r] * v.value[0] +
				value[1][r] * v.value[1] +
				value[2][r] * v.value[2];
	}
	return o;
}

inline Mat3 Mat3::operator*(float s) const
{
	Mat3 o;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			o.value[r][c] = value[r][c] * s;
		}
	}
	return o;
}

inline Mat3 Mat3::operator/(float s) const
{
	Mat3 o;
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			o.value[r][c] = value[r][c] / s;
		}
	}
	return o;
}

inline void Mat3::setZero()
{
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			value[r][c] = 0;
		}
	}
}

inline void Mat3::setIdentity()
{
	value[0][0] = 1;
	value[1][1] = 1;
	value[2][2] = 1;
	value[0][1] = 0;
	value[0][2] = 0;
	value[1][0] = 0;
	value[1][2] = 0;
	value[2][0] = 0;
	value[2][1] = 0;
}

inline Mat3 Mat3::inverse()
{
	float det = value[0][0] * value[1][1] * value[2][2] +
		value[1][0] * value[2][1] * value[0][2] +
		value[2][0] * value[0][1] * value[1][2] -
		value[0][0] * value[1][2] * value[2][1] -
		value[0][1] * value[1][0] * value[2][2] -
		value[0][2] * value[1][1] * value[2][0];

	Mat3 inv;
	inv.value[0][0] = (value[1][1] * value[2][2] - value[1][2] * value[2][1]) / det;
	inv.value[0][1] = -(value[1][0] * value[2][2] - value[1][2] * value[2][0]) / det;
	inv.value[0][2] = (value[1][0] * value[2][1] - value[1][1] * value[2][0]) / det;
	inv.value[1][0] = -(value[0][1] * value[2][2] - value[0][2] * value[2][1]) / det;
	inv.value[1][1] = (value[0][0] * value[2][2] - value[0][2] * value[2][0]) / det;
	inv.value[1][2] = -(value[0][0] * value[2][1] - value[0][1] * value[2][0]) / det;
	inv.value[2][0] = (value[0][1] * value[1][2] - value[0][2] * value[1][1]) / det;
	inv.value[2][1] = -(value[0][0] * value[1][2] - value[0][2] * value[1][0]) / det;
	inv.value[2][2] = (value[0][0] * value[1][1] - value[0][1] * value[1][0]) / det;
	return inv;
}

inline Mat3 Mat3::transpose()
{
	Mat3 t;
	t.value[0][0] = value[0][0];
	t.value[1][1] = value[1][1];
	t.value[2][2] = value[2][2];
	t.value[0][1] = value[1][0];
	t.value[0][2] = value[2][0];
	t.value[1][0] = value[0][1];
	t.value[1][2] = value[2][1];
	t.value[2][0] = value[0][2];
	t.value[2][1] = value[1][2];
	return t;
}
