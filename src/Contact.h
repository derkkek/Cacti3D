#pragma once
#include <vector>
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

		float timeOfImpact;
		float seperationDistance;
	};

	void ResolveContact(Contact& c);
	int CompareContacts(const Contact& c0, const Contact& c1);
	void SortContactsByTheirTimeOfImpact(std::vector<Contact>& contacts);
}