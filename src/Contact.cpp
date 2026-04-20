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

		a->linearVelocity/= 2;
	}
}