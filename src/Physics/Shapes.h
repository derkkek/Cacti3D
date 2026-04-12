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
		Shape() = default;
		~Shape() = default;

		const virtual ShapeType GetType() = 0;

	private:

	};

	class Sphere : public Shape
	{
	public:
		Sphere(float radius)
			:radius(radius)
		{

		}
		~Sphere() = default;

		const ShapeType GetType()
		{
			return SPHERE;
		}

		float radius;
	private:

	};

}