#pragma once

#include "util/vectorbase.h"
#include "util/quaternion.h"

class Mat3 {
public:
	float value[3][3];
	// Constructor
	inline Mat3(void) {
		setZero();
	}
	// Copy-Constructor
	inline Mat3(const Mat3& m) {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				value[r][c] = m.value[r][c];
			}
		}
	}

	Mat3(const GamePhysics::Mat4& m) {
		for (int c = 0; c < 3; c++) {
			value[0][c] = m.value[0][c];
			value[1][c] = m.value[1][c];
			value[2][c] = m.value[2][c];
		}
	}
	Mat3(const GamePhysics::Quat& q) {
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
	// Assignment operator
	inline const Mat3& operator=  (const Mat3& m) {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				value[r][c] = m.value[r][c];
			}
		}
		return *this;
	}
	// Assign and mult operator
	inline const Mat3& operator*= (float s) {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				value[r][c] *= s;
			}
		}
		return *this;
	}
	// Assign and div operator
	inline const Mat3& operator/= (float s) {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				value[r][c] /= s;
			}
		}
		return *this;
	}

	// unary operator
	inline Mat3 operator- () const {
		Mat3 o;
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				o.value[r][c] = -value[r][c];
			}
		}
		return o;
	}

	// binary operator add
	inline Mat3 operator+ (const Mat3& m) const {
		Mat3 o;
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				o.value[r][c] = value[r][c] + m.value[r][c];
			}
		}
		return o;
	}
	// binary operator sub
	inline Mat3 operator- (const Mat3& m) const
	{
		Mat3 o;
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				o.value[r][c] = value[r][c] - m.value[r][c];
			}
		}
		return o;
	}
	// binary operator mult
	inline Mat3 operator* (const Mat3& m) const {
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
	// binary operator mult
	inline GamePhysics::Vec3 dot(const GamePhysics::Vec3& v) const {
		GamePhysics::Vec3 o;
		for (int r = 0; r < 3; r++) {
			o.value[r] =
				value[r][0] * v.value[0] +
				value[r][1] * v.value[1] +
				value[r][2] * v.value[2];
		}
		return o;
	}
	// binary operator mult
	inline Mat3 operator* (float s) const {
		Mat3 o;
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				o.value[r][c] = value[r][c] * s;
			}
		}
		return o;
	}
	// binary operator div
	inline Mat3 operator/ (float s) const {
		Mat3 o;
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				o.value[r][c] = value[r][c] / s;
			}
		}
		return o;
	}

	inline void setZero() {
		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				value[r][c] = 0;
			}
		}
	}
	inline void setIdentity() {
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
	inline Mat3 inverse() {
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

	inline Mat3 transpose()
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
};
