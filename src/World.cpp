#include "World.h"
#include "Intersections.h"
#include <iostream>
#include <Contact.h>
#include <algorithm>
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
		contacts.resize(MaxBodies * MaxBodies);
		bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(-20, 3, 0), Vec3(10,0,0), Vec3(0, 0, 0), 1.0f, 0.5f, 1.0f);
		bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(20, 3, 0), Vec3(-10, 0, 0), Vec3(0, 0, 0), 1.0f, 0.5f, 1.0f);


		bodies.emplace_back(std::make_unique<Sphere>(1000), Vec3(0, -1000, 0));

	}
	void World::Update(float dt)
	{
		for (int i = 0; i < bodies.size(); i++)
		{
			bodies[i].ApplyImpulse(bodies[i].position, Vec3(0, -10, 0) * dt);
		}

		numContacts = 0;

		for (int i = 0; i < bodies.size(); i++)
		{
			for (int j = i + 1; j < bodies.size(); j++)
			{
				Contact contact{};

				if (bodies[i].invMass == 0 && bodies[j].invMass == 0)
				{
					continue;
				}

				if (Intersections::Intersect(bodies[i], bodies[j], contact, dt))
				{
					contacts[numContacts] = contact;
					numContacts++;
				}

			}
		}

		if (numContacts > 1)
		{
			SortContactsByTheirTimeOfImpact(contacts);
		}
		
		float accumulatedTime = 0.0f;
		for (int i = 0; i < numContacts; i++) 
		{
			Contact& contact = contacts[i];

			float passedTimeToCollision = contact.timeOfImpact - accumulatedTime;


			if (contact.a->invMass == 0 && contact.b->invMass == 0)
			{
				continue;
			}

			for (int j = 0; j < bodies.size(); j++) 
			{
				bodies[j].Update(passedTimeToCollision);
			}

			ResolveContact(contact);

			accumulatedTime += passedTimeToCollision;
		}

		float timeRemaining = dt - accumulatedTime;
		if (timeRemaining > 0.0f) {
			for (int i = 0; i < bodies.size(); i++) {
				bodies[i].Update(timeRemaining);
			}
		}

	}
}