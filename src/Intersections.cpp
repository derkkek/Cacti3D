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

			if (baMagSquared <= ((sphereA->radius + sphereB->radius) * (sphereA->radius + sphereB->radius)))
			{
				contact.a = &bodyA;
				contact.b = &bodyB;
				Vec3 ab = bodyB.position - bodyA.position;
				contact.normal = ab.Normalize();


				// TODO: -->
				Vec3 worldCollisionPointA = bodyA.position + contact.normal * sphereA->radius;
				Vec3 worldCollisionPointB = bodyB.position - contact.normal * sphereB->radius;

				contact.localPointA = bodyA.WorldSpaceToLocalSpace(worldCollisionPointA);
				contact.localPointB = bodyB.WorldSpaceToLocalSpace(worldCollisionPointB);
				return true;
			}

		}
		return false;
	}
}