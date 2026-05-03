#include "Intersections.h"
#include "Physics/Shapes.h"
#include "Math/Vector.h"
#include <Contact.h>

namespace Cacti
{
	bool Intersections::Intersect(Body& bodyA, Body& bodyB, Contact& contact, const float dt)
	{
		if (bodyA.shape.get()->GetType() == SPHERE && bodyB.shape.get()->GetType() == SPHERE)
		{
			const Sphere* sphereA = reinterpret_cast<const Sphere*>(bodyA.shape.get());
			const Sphere* sphereB = reinterpret_cast<const Sphere*>(bodyB.shape.get());

			float t0(0);
			float t1(0);

			const Vec3 vA = bodyA.linearVelocity;
			const Vec3 vB = bodyB.linearVelocity;
			if (SphereSphereDynamic(sphereA, sphereB, vA, vB, bodyA.position, bodyB.position, dt, t0, t1, contact.timeOfImpact, contact.worldPointA, contact.worldPointB))
			{
				//save future contact data.
				bodyA.Update(contact.timeOfImpact);
				bodyB.Update(contact.timeOfImpact);

				contact.localPointA = bodyA.WorldSpaceToLocalSpace(contact.worldPointA);
				contact.localPointB = bodyB.WorldSpaceToLocalSpace(contact.worldPointB);

				const Vec3 ba = bodyA.position - bodyB.position;
				contact.normal = ba.Normalized();
				
				//revert to original pos.
				bodyA.Update(-contact.timeOfImpact);
				bodyB.Update(-contact.timeOfImpact);

				const Vec3 ab = bodyB.position - bodyA.position;
				float r = ab.GetMagnitude() - (sphereA->radius + sphereB->radius);
				contact.seperationDistance = r;
				return true;
			}
		
		}

		// TODO: either populate world points in here by calling local space to world space func or in resolution. Currently it's being populated only for spheres.
		return false;
	}
	bool Intersections::RayTraceCollidedWithSphere(const Vec3& rayOrigin, const Vec3& rayDir, const Vec3& sphereCenter, float& sphereRadius, float& t1, float& t2)
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

	bool Intersections::SphereSphereDynamic(const Sphere* sphereA, const Sphere* sphereB, const Vec3& vA, const Vec3& vB, const Vec3& aPos, const Vec3& bPos, const float dt, float& t0, float& t1, float& toi, Vec3& collisionPointOnA, Vec3& collisionPointOnB)
	{
		const Vec3 relV = vA - vB; // let's say B is static.
		const Vec3 rayOrigin = aPos;
		const Vec3 rayEnd = aPos + relV * dt;
		const Vec3 rayVector = rayEnd - rayOrigin;

		float radiusSum = sphereA->radius + sphereB->radius + 0.001f;


		if (rayVector.GetLengthSqr() <= 0.001f * 0.001f)
		{
			//we're so close are we colliding right now?

			const Vec3 ab = bPos - aPos;
			if (ab.GetLengthSqr() > radiusSum * radiusSum)
			{
				return false;
			}
		}

		else if (!RayTraceCollidedWithSphere(rayOrigin, rayVector, bPos, radiusSum, t0, t1))
		{
			return false;
		}

		//change our scale from [0, 1] to [0, dt], (dt is the time of the final position of the ray.)
		//Because of this we can sort collision times and handle collisions chronogically otherwise we could miss prior collisions.
		//Also while calculating new positions we will use dt for our toi. Using [0,1] scale is useless.
		t0 *= dt;
		t1 *= dt;

		if (t1 < 0)
		{
			//Collision is in the past, no future collisions in this frame.
			return false;
		}

		//earliest time of impact
		toi = (t0 < 0.0f) ? 0.0f : t0;

		if (toi > dt)
		{
			//earliest collision is not in this frame, it's in future.
			return false;
		}

		const Vec3 newPositionA = aPos + vA * toi;
		const Vec3 newPositionB = bPos + vB * toi;

		const Vec3 collisionNormal = (newPositionA - newPositionB).Normalize();

		collisionPointOnA = newPositionA + collisionNormal * sphereA->radius;
		collisionPointOnB = newPositionB - collisionNormal * sphereB->radius;

		return true;
	}

}