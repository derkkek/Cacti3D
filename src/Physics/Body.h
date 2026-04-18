#pragma once
#include "../Math/Vector.h"
#include "../Math/Quaternion.h"
#include "Shapes.h"
#include <memory>
namespace Cacti
{
	class Body
	{
	public:
		Body(std::unique_ptr<Shape> shape, Vec3 position, Vec3 angularVel = Vec3(0,0,0));

		void Update(float dt);
		Vec3 position;

		Vec3 angularVelocity;
		Quaternion orientation;
		std::unique_ptr<Shape> shape;

		Vec3 WorldSpaceToLocalSpace(const Vec3 p);
	private:
	};

}