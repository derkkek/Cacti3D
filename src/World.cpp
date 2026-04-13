#include "World.h"
#include "Intersections.h"
#include <iostream>
#include <Contact.h>
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
		bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(0, 0.5, 0));
		bodies.emplace_back(std::make_unique<Sphere>(100), Vec3(0, -100, 0));

	}
	void World::Update(float dt)
	{
		bodies[0].position.x -= 1 * dt;
		contacts.clear();

		for (int i = 0; i < bodies.size(); i++)
		{
			for (int j = i + 1; j < bodies.size(); j++)
			{
				Contact contact{};
				if (Intersections::Intersect(bodies[i], bodies[j], contact))
				{
					contacts.emplace_back(contact);
				}

			}
		}
	}
}