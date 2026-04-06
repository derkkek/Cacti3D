#pragma once
#include "Physics/Body.h"
#include <vector>
namespace Cacti
{
	class World
	{
	public:
		World();
		~World();

		void Init();
		//void AddBody();
		std::vector<Body> bodies;

		const int MaxBodies = 100;

	private:

	};

}