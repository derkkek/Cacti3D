#pragma once
#include "Vector.h"
#include <vector>
namespace Cacti
{
	struct Triangle
	{
		int a;
		int b;
		int c;
	};

	struct Tetrahedron
	{
		std::vector<Triangle> triangles;
		std::vector<Vec3> vertices;
	};
}