#include "World.h"

namespace Cacti
{
	World::World()
	{
		bodies.emplace_back(new Sphere(5),Vec3(0, 0, 0));
	}
	World::~World()
	{
	}
	void World::Init()
	{
		bodies.resize(MaxBodies);
		bodies.emplace_back(Vec3(0, 0, 0));
	}
}