#pragma once

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

		float radius;
	private:

	};

}