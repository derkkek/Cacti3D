#pragma once
#include "Math/Vector.h"
#include "Physics/Body.h"

namespace Cacti
{
	struct Contact
	{
		Vec3 localPointA;
		Vec3 localPointB;
		Vec3 worldPointA;
		Vec3 worldPointB;

		Vec3 normal;

		Body* a;
		Body* b;
	};

	void ResolveContact(Contact& c);
}