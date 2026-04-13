#pragma once
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "Shapes.h"
#include <memory>
namespace Cacti
{
	class Body
	{
	public:
		Body(std::unique_ptr<Shape> shape, Vec3 position);

		Vec3 position;
		Quat orientation;
		std::unique_ptr<Shape> shape;

		Vec3 WorldSpaceToLocalSpace(const Vec3 p);
	private:
	};

}