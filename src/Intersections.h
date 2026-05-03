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

		static bool Intersect(Body& bodyA, Body& bodyB, Contact& contact, const float dt);
		static bool RayTraceCollidedWithSphere(const Vec3& rayOrigin, const Vec3& rayDir, const Vec3& sphereCenter, float& sphereRadius, float& t1, float& t2);
		static bool SphereSphereDynamic(const Sphere* sphereA, const Sphere* sphereB, const Vec3& vA, const Vec3& vB, const Vec3& aPos, const Vec3& bPos, const float dt, float& t0, float& t1, float& toi, Vec3& collisionPointOnA, Vec3& collisionPointOnB);
	private:

	};
}