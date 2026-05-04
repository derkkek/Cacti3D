#include "Contact.h"
#include <iostream>
#include <algorithm>

namespace Cacti
{
	void ResolveContact(Contact& c)
	{
		Body* a = c.a;
		Body* b = c.b;

		float combinedE = a->e * b->e;

		const Vec3 pA = a->LocalSpaceToWorldSpace(c.localPointA);
		const Vec3 pB = b->LocalSpaceToWorldSpace(c.localPointB);

		const Vec3 r1 = pA - a->CenterOfMassWorldSpace();
		const Vec3 u1 = a->linearVelocity + a->angularVelocity.Cross(r1);

		const Vec3 r2 = pB - b->CenterOfMassWorldSpace();
		const Vec3 u2 = b->linearVelocity + b->angularVelocity.Cross(r2);

		const Vec3 relV = u1 - u2;

		const Vec3 angularFactorA = (a->GetInverseInertiaWorldSpace() * r1.Cross(c.normal)).Cross(r1);
		const Vec3 angularFactorB = (b->GetInverseInertiaWorldSpace() * r2.Cross(c.normal)).Cross(r2);

		const float angularFactor = (angularFactorA + angularFactorB).Dot(c.normal);

		const float projectedVelOntoCollisionNormal = relV.Dot(c.normal);
		
		const float JMag = (projectedVelOntoCollisionNormal * (1 + combinedE)) / ((a->invMass + b->invMass) + angularFactor);

		const Vec3 J = c.normal * JMag;

		a->ApplyImpulse(pA, J * -1.0f);
		b->ApplyImpulse(pB, J);

		//frictional-tangential impulse

		const float frictionA = a->friction;
		const float frictionB = b->friction;
		const float friction = frictionA * frictionB;

		const Vec3 collisionVelocityOnNormalDirection = c.normal * c.normal.Dot(relV);

		//substract velocity on normal direction so that we can end up with the only velocity on tangential direction.
		const Vec3 collisionTangentialVelocity = relV - collisionVelocityOnNormalDirection;

		//think this as like c.normal for friction impulse.
		const Vec3 normalOfCollisionTangentialVelocity = collisionTangentialVelocity.Normalized();

		const Vec3 inertiaA = (a->GetInverseInertiaWorldSpace() * r1.Cross(normalOfCollisionTangentialVelocity)).Cross(r1);
		const Vec3 inertiaB = (b->GetInverseInertiaWorldSpace() * r2.Cross(normalOfCollisionTangentialVelocity)).Cross(r2);
		const float invInertia = (inertiaA + inertiaB).Dot(normalOfCollisionTangentialVelocity);

		const float reducedMass = 1.0f / (a->invMass + b->invMass + invInertia);
		const Vec3 impulseFriction = collisionTangentialVelocity * reducedMass * friction;

		a->ApplyImpulse(c.worldPointA, impulseFriction * -1.0f);
		b->ApplyImpulse(c.worldPointB, impulseFriction);


		if (c.timeOfImpact == 0.0f)
		{
			const float tA = a->invMass / (a->invMass + b->invMass);
			const float tB = b->invMass / (a->invMass + b->invMass);
			const Vec3 ds = pB - pA;
			a->position += ds * tA;
			b->position -= ds * tB;
		}

	}
	int CompareContacts(const Contact& c0, const Contact& c1)
	{
		if (c0.timeOfImpact < c1.timeOfImpact)
		{
			return -1;
		}
		else if (c0.timeOfImpact == c1.timeOfImpact)
		{
			return 0;
		}

		return 1;
	}
	void SortContactsByTheirTimeOfImpact(std::vector<Contact>& contacts)
	{
		std::sort(contacts.begin(), contacts.begin() + contacts.size(),
			[](const Contact& a, const Contact& b) { return CompareContacts(a, b) < 0; });
	}
}