#pragma once
#include "../Math/Vector.h"
#include "Shapes.h"
namespace Cacti
{
	class Body
	{
	public:
		Body(Shape* shape, Vec3 position);
		~Body();

		Vec3 position;
		Shape* shape;
	private:
	};

}