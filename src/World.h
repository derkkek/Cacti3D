#pragma once
#include "Physics/Body.h"
#include <vector>
#include "Contact.h"
#include "Physics/Broadphase.h"

namespace Cacti
{
	class World
	{
	public:
		World();
		~World();

		void Init();
		void Update(float dt);

		const int MaxBodies = 500;


		Broadphase broadPhase;


		std::vector<Body> bodies;
		std::vector<Contact> contacts;

		int numContacts = 0;
		int numCollisionPairs = 0;

		float time = 0;
	private:

	};

}