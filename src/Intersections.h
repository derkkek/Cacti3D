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
		static bool RayTraceCollidedWithSphere(Vec3& rayOrigin, Vec3& rayDir, Vec3& sphereCenter, float& sphereRadius, float& t1, float& t2);
	private:

	};
}