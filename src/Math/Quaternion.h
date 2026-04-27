//
//	Quaternion.h
//
#pragma once
#include "Vector.h"
#include "Matrix.h"

/*
 ================================
 Quaternion
 ================================
 */
namespace Cacti
{
	class Quaternion {
	public:
		Quaternion();
		Quaternion(const Quaternion& rhs);
		Quaternion(float X, float Y, float Z, float W);
		Quaternion(Vec3 n, const float angleRadians);
		const Quaternion& operator = (const Quaternion& rhs);

		Quaternion& operator *= (const float& rhs);
		Quaternion& operator *= (const Quaternion& rhs);
		Quaternion	operator * (const Quaternion& rhs) const;

		void ToAxisAngle(Vec3& axis, float& angle) const;

		void	Normalize();
		void	Invert();
		Quaternion	Inverse() const;
		float	MagnitudeSquared() const;
		float	GetMagnitude() const;
		Vec3	RotatePoint(const Vec3& rhs) const;
		Vec3	RotatePointOptimized(const Vec3& rhs) const;


		Mat3	RotateMatrix(const Mat3& rhs) const;
		Vec3	xyz() const { return Vec3(x, y, z); }
		bool	IsValid() const;

		Mat3	ToMat3() const;
		Vec4	ToVec4() const { return Vec4(w, x, y, z); }

	public:
		float w;
		float x;
		float y;
		float z;
	};

	inline Quaternion::Quaternion() :
		x(0),
		y(0),
		z(0),
		w(1) {
	}

	inline Quaternion::Quaternion(const Quaternion& rhs) :
		x(rhs.x),
		y(rhs.y),
		z(rhs.z),
		w(rhs.w) {
	}

	inline Quaternion::Quaternion(float X, float Y, float Z, float W) :
		x(X),
		y(Y),
		z(Z),
		w(W) {
	}

	inline Quaternion::Quaternion(Vec3 n, const float angleRadians)
	{
		const float halfAngleRadians = 0.5f * angleRadians;

		w = cosf(halfAngleRadians);

		const float halfSine = sinf(halfAngleRadians);
		n.Normalize();
		x = n.x * halfSine;
		y = n.y * halfSine;
		z = n.z * halfSine;
	}

	inline const Quaternion& Quaternion::operator = (const Quaternion& rhs) {
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		w = rhs.w;
		return *this;
	}

	inline Quaternion& Quaternion::operator *= (const float& rhs) {
		x *= rhs;
		y *= rhs;
		z *= rhs;
		w *= rhs;
		return *this;
	}

	inline Quaternion& Quaternion::operator *= (const Quaternion& rhs) {
		Quaternion temp = *this * rhs;
		w = temp.w;
		x = temp.x;
		y = temp.y;
		z = temp.z;
		return *this;
	}

	inline Quaternion Quaternion::operator * (const Quaternion& rhs) const {
		Quaternion temp;
		temp.w = (w * rhs.w) - (x * rhs.x) - (y * rhs.y) - (z * rhs.z);
		temp.x = (x * rhs.w) + (w * rhs.x) + (y * rhs.z) - (z * rhs.y);
		temp.y = (y * rhs.w) + (w * rhs.y) + (z * rhs.x) - (x * rhs.z);
		temp.z = (z * rhs.w) + (w * rhs.z) + (x * rhs.y) - (y * rhs.x);
		return temp;
	}

	inline void Quaternion::ToAxisAngle(Vec3& axis, float& angle) const
	{
		angle = 2.0f * acosf(w);

		float sinHalfAngle = sqrtf(1.0f - w * w);

		if (sinHalfAngle < 0.0001f)
		{
			axis = Vec3(0.0f, 1.0f, 0.0f);
		}
		else
		{
			axis = Vec3(x / sinHalfAngle, y / sinHalfAngle, z / sinHalfAngle);
		}
	}

	inline void Quaternion::Normalize() {
		float invMag = 1.0f / GetMagnitude();

		if (0.0f * invMag == 0.0f * invMag) {
			x = x * invMag;
			y = y * invMag;
			z = z * invMag;
			w = w * invMag;
		}
	}

	inline void Quaternion::Invert() {
		*this *= 1.0f / MagnitudeSquared();
		x = -x;
		y = -y;
		z = -z;
	}

	inline Quaternion Quaternion::Inverse() const {
		Quaternion val(*this);
		val.Invert();
		return val;
	}

	inline float Quaternion::MagnitudeSquared() const {
		return ((x * x) + (y * y) + (z * z) + (w * w));
	}

	inline float Quaternion::GetMagnitude() const {
		return sqrtf(MagnitudeSquared());
	}

	inline Vec3 Quaternion::RotatePoint(const Vec3& rhs) const {
		Quaternion vector(rhs.x, rhs.y, rhs.z, 0.0f);
		Quaternion final = *this * vector * Inverse();
		return Vec3(final.x, final.y, final.z);
	}

	/*If quaternion is normalized, use this, this doesn't divide.*/
	inline Vec3 Quaternion::RotatePointOptimized(const Vec3& rhs) const
	{
		Quaternion conjugate = Quaternion(-x, -y, -z, w);
		Quaternion v = Quaternion(rhs.x, rhs.y, rhs.z, 0);
		Quaternion result = (*this) * v * conjugate;
		return Vec3(result.x, result.y, result.z);
	}

	inline bool Quaternion::IsValid() const {
		if (x * 0 != x * 0) {
			return false;
		}
		if (y * 0 != y * 0) {
			return false;
		}
		if (z * 0 != z * 0) {
			return false;
		}
		if (w * 0 != w * 0) {
			return false;
		}
		return true;
	}

	inline Mat3 Quaternion::RotateMatrix(const Mat3& rhs) const {
		Mat3 mat;
		mat.rows[0] = RotatePoint(rhs.rows[0]);
		mat.rows[1] = RotatePoint(rhs.rows[1]);
		mat.rows[2] = RotatePoint(rhs.rows[2]);
		return mat;
	}

	inline Mat3 Quaternion::ToMat3() const {
		Mat3 mat;
		mat.Identity();

		mat.rows[0] = RotatePoint(mat.rows[0]);
		mat.rows[1] = RotatePoint(mat.rows[1]);
		mat.rows[2] = RotatePoint(mat.rows[2]);
		return mat;
	}
}
