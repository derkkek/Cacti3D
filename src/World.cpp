#include "World.h"
#include "Intersections.h"
#include <iostream>
#include <Contact.h>
#include <algorithm>
#include "Physics/Broadphase.h"
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
		contacts.resize(MaxBodies * MaxBodies, Contact{});
	/*	bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(-100, 3, 0), Vec3(500,0,0), Vec3(0, 0, 0), 1.0f, 0.5f, 1.0f);
		bodies.emplace_back(std::make_unique<Sphere>(1), Vec3(100, 3, 0), Vec3(0, 0, 0), Vec3(0, 0, 0), 1.0f, 0.5f, 1.0f);*/
		for(int x = 0; x < 5; x++) 
		{
			for(int z = 0; z < 5; z++) 
			{
				for (int y = 0; y < 5; y++)
				{
					float radius = 0.5f;
					float xx = float(x - 1) * radius * 3.0f;
					float zz = float(z - 1) * radius * 3.0f;
					float yy = float(y - 1) * radius * 6.0f;
					bodies.emplace_back(std::make_unique<Sphere>(radius), Vec3(xx, yy, zz), Vec3(0, 0, 0), Vec3(0, 0, 0), 1.0f, 0.5f, 1.0f);

				}

			
			}
		}


		bodies.emplace_back(std::make_unique<Sphere>(200), Vec3(0, -200, 0));

	}
	void World::Update(float dt)
	{
		for (int i = 0; i < bodies.size(); i++)
		{
			bodies[i].ApplyImpulse(bodies[i].position, Vec3(0, -10, 0) * dt);
		}

		numContacts = 0;

		std::vector<Broadphase::CollisionPair> collisionPairs = Broadphase::SweepAndPrune(bodies);

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