#pragma once
#include "Physics/Body.h"
#include "Contact.h"
namespace Cacti
{
	class Intersections
	{
	public:
		Intersections() = default;
		~Intersections() = default;

		static bool Intersect(Body& bodyA, Body& bodyB, Contact& contact);
	private:

	};
}