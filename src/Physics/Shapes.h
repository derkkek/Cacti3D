#pragma once
#include "Math/Matrix.h"
#include <Math/AABB.h>
#include <Math/Quaternion.h>

namespace Cacti
{
	enum ShapeType
	{
		SPHERE
	};

	class Shape
	{
	public:
		Shape(Vec3 com)
			:centerOfMass(com)
		{

		}
		~Shape() = default;

		const virtual ShapeType GetType() = 0;

		virtual Vec3 GetCenterOfMass() const
		{
			return centerOfMass;
		}

		virtual Mat3 GetInertiaTensor() const = 0;

		virtual AABB GetAABBWorldSpace(const Vec3& pos, const Quaternion& orient) = 0;

	protected:
		Vec3 centerOfMass;
	};

	class Sphere : public Shape
	{
	public:
		Sphere(float radius)
			:radius(radius), Shape(Vec3(0,0,0))
		{

		}
		~Sphere() = default;

		const ShapeType GetType()
		{
			return SPHERE;
		}

		Vec3 GetCenterOfMass() const override
		{
			return Vec3(0,0,0);
		}

		Mat3 GetInertiaTensor() const override
		{
			Mat3 tensor{};
			tensor.Zero();
			tensor.rows[0][0] = 2.0f / 5.0f * radius * radius;
			tensor.rows[1][1] = 2.0f / 5.0f * radius * radius;
			tensor.rows[2][2] = 2.0f / 5.0f * radius * radius;
			return tensor;
		}

		AABB GetAABBWorldSpace(const Vec3& pos, const Quaternion& orient) override
		{
			AABB aabb{};
			aabb.min = Vec3(-radius) + pos;
			aabb.max = Vec3(radius) + pos;
			return aabb;
		}

		float radius;
	private:

	};

}