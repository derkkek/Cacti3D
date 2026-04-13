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
		//bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(0, 3, 0));
		bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(3, 1, 0));
		bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(5, 1, 0));
		bodies.emplace_back(std::make_unique<Sphere>(100), Vec3(0, -100, 0));

	}
	void World::Update(float dt)
	{
		time += dt;
		//bodies[0].position.y = sinf(time) + 0.5;
		//bodies[0].position.x = sinf(time * 2);
		bodies[0].position.y = sinf(time * 2) + 1;
		bodies[0].position.x = sinf(time) * 2 + 3;
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