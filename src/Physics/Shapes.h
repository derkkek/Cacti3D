#pragma once
#include "Math/Matrix.h"

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
			tensor.rows[0][0] = 2 / 5 * radius * radius;
			tensor.rows[1][1] = 2 / 5 * radius * radius;
			tensor.rows[2][2] = 2 / 5 * radius * radius;
			return tensor;
		}

		float radius;
	private:

	};

}