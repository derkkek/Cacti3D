#include "Contact.h"
#include <iostream>

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
		
		const float JMag = (projectedVelOntoCollisionNormal * -(1 + combinedE)) / ((a->invMass + b->invMass) + angularFactor);

		const Vec3 J = c.normal * JMag;

		a->ApplyImpulse(pA, J);
		b->ApplyImpulse(pB, J * -1.0f);

		const float tA = a->invMass / (a->invMass + b->invMass);
		const float tB = b->invMass / (a->invMass + b->invMass);

		const Vec3 ds = pB - pA;
		a->position += ds * tA;
		b->position -= ds * tB;
	}
}