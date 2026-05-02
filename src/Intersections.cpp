#include "Intersections.h"
#include "Physics/Shapes.h"
#include "Math/Vector.h"
#include <Contact.h>

namespace Cacti
{
	bool Intersections::Intersect(Body& bodyA, Body& bodyB, Contact& contact)
	{
		if (bodyA.shape.get()->GetType() == SPHERE && bodyB.shape.get()->GetType() == SPHERE)
		{
			const Sphere* sphereA = reinterpret_cast<const Sphere*>(bodyA.shape.get());
			const Sphere* sphereB = reinterpret_cast<const Sphere*>(bodyB.shape.get());

			const Vec3 baVector = bodyA.position - bodyB.position;

			const float baMagSquared = baVector.GetLengthSqr();
			Vec3 rayO = sphereA->GetCenterOfMass();
			Vec3 rayD = sphereB - sphereA;
			Vec3 sphereCenter = sphereB->GetCenterOfMass();
			float radius = sphereA->radius + sphereB->radius;
			float t1;
			float t2;
			if (RayTraceCollidedWithSphere(rayO, rayD, sphereCenter, radius, t1, t2))
			{
				if (t1 < 0 && t2 < 0)
				{
					return false;
				}
				else if ((t1 < t2) && 0.0f < t1 && t1 < 1.0f)
				{
					bodyA.Update(t1);
					bodyB.Update(t1);
				}
				else if ((t2 < t1) && 0.0f < t2 && t2 < 1.0f)
				{
					bodyA.Update(t2);
					bodyB.Update(t2);
				}
				if (baMagSquared <= ((radius) * (radius)))
				{
					contact.a = &bodyA;
					contact.b = &bodyB;
					Vec3 ab = bodyB.position - bodyA.position;
					contact.normal = ab.Normalize();

					Vec3 worldCollisionPointA = bodyA.position + contact.normal * sphereA->radius;
					Vec3 worldCollisionPointB = bodyB.position - contact.normal * sphereB->radius;

					contact.worldPointA = worldCollisionPointA;
					contact.worldPointB = worldCollisionPointB;

					contact.localPointA = bodyA.WorldSpaceToLocalSpace(worldCollisionPointA);
					contact.localPointB = bodyB.WorldSpaceToLocalSpace(worldCollisionPointB);

					return true;
				}
			}
		
		}

		// TODO: either populate world points in here by calling local space to world space func or in resolution. Currently it's being populated only for spheres.
		return false;
	}
	bool Intersections::RayTraceCollidedWithSphere(Vec3& rayOrigin, Vec3& rayDir, Vec3& sphereCenter, float& sphereRadius, float& t1, float& t2)
	{
		float a = rayDir.Dot(rayDir);
		float b = 2 * rayDir.Dot(rayOrigin - sphereCenter);
		float c = (rayOrigin - sphereCenter).Dot(rayOrigin - sphereCenter) - sphereRadius * sphereRadius;

		float discriminant = b * b - 4 * a * c;

		if (discriminant < 0)
		{
			return false;
		}

		t1 = (-b - sqrtf(discriminant)) / (2 * a);
		t2 = (-b + sqrtf(discriminant)) / (2 * a);

		return true;
	}

}