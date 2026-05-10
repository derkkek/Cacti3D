#include "World.h"
#include "Intersections.h"
#include <iostream>
#include <Contact.h>
#include <algorithm>
#include "Physics/Broadphase.h"
namespace Cacti
{
	World::World()
		:MaxBodies(700), broadPhase(MaxBodies)
	{
	}
	World::~World()
	{
	}
	void World::Init()
	{
		bodies.reserve(MaxBodies);
		contacts.resize(MaxBodies * 4, Contact{});

		//bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(-100, 3, 0), Vec3(2000,0,0), Vec3(0, 0, 0), 1.0f, 0.5f, 1.0f);
		//bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(0, 3, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), 1.0f, 0.5f, 1.0f);
		bodies.emplace_back(std::make_unique<Box>(5.0f, 5.0f, 5.0f), Vec3(0, 10, 0));
		//bodies.emplace_back(std::make_unique<Sphere>(1000), Vec3(0, -1000, 0));

	}
	void World::Update(float dt)
	{
		for (int i = 0; i < bodies.size(); i++)
		{
			bodies[i].ApplyImpulse(bodies[i].position, Vec3(0, -10, 0) * dt);
		}

		numContacts = 0;

		std::vector<Broadphase::CollisionPair>& collisionPairs = broadPhase.SweepAndPrune(bodies, dt);

		for (int i = 0; i < collisionPairs.size(); i++)
		{
			Contact contact{};
			Broadphase::CollisionPair pair = collisionPairs[i];

			if (pair.a->invMass == 0 && pair.b->invMass == 0)
			{
				continue;
			}
			if (Intersections::Intersect(*pair.a, *pair.b, contact, dt))
			{
				contacts[numContacts] = contact;
				numContacts++;

			}
		}
		

		if (numContacts > 1)
		{
			SortContactsByTheirTimeOfImpact(contacts, numContacts);
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