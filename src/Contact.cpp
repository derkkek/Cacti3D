#include "Contact.h"

namespace Cacti
{
	void ResolveContact(Contact& c)
	{
		Body* a = c.a;
		Body* b = c.b;

		const float tA = a->invMass / (a->invMass + b->invMass);
		const float tB = b->invMass / (a->invMass + b->invMass);

		const Vec3 ds = c.worldPointB - c.worldPointA;
		a->position += ds * tA;
		b->position -= ds * tB;

		float combinedE = a->e * b->e;
		const Vec3 relV = a->linearVelocity - b->linearVelocity;
		const float projectedVelOntoCollisionNormal = relV.Dot(c.normal);
		
		float JMag = (projectedVelOntoCollisionNormal * -(1 + combinedE)) / (a->invMass + b->invMass);

		a->ApplyImpulse(c.normal * JMag);
		b->ApplyImpulse(c.normal * -JMag);
	}
}