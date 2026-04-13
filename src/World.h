#pragma once
#include "Physics/Body.h"
#include <vector>
#include "Contact.h"
namespace Cacti
{
	class World
	{
	public:
		World();
		~World();

		void Init();
		void Update(float dt);

		std::vector<Body> bodies;
		std::vector<Contact> contacts;

		const int MaxBodies = 100;

		float time = 0;
	private:

	};

}