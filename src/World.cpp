#include "World.h"

namespace Cacti
{
	World::World()
	{
	}
	World::~World()
	{
	}
	void World::Init()
	{
		bodies.reserve(MaxBodies);
		bodies.emplace_back(new Sphere(5), Vec3(0, 0, 0));

	}
}