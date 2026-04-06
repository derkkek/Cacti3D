#pragma once
#include <vector>
#include "Math/Vector.h"
namespace Cacti
{
	struct TransformBuffer
	{
		std::vector<Vec3> positions;


		void Init(int size)
		{
			positions.resize(size);
		}
	};
}
