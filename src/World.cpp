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
		bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(0, 3, 0));
		bodies.emplace_back(std::make_unique<Sphere>(100), Vec3(0, -100, 0));

	}
	void World::Update(float dt)
	{
		for (int i = 0; i < bodies.size(); i++)
		{
			bodies[i].position.x += 1 * dt;
		}
	}
}