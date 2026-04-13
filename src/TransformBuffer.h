#pragma once
#include <vector>
#include "Math/Vector.h"
#include "Math/Quat.h"
#include <Contact.h>
namespace Cacti
{
	struct TransformBuffer
	{
		std::vector<Vec3> positions;
		std::vector<Quat> orientations;
		void Init(int size)
		{
			positions.resize(size);
			orientations.resize(size);
		}
	};

	struct ContactBuffer
	{
		std::vector<Contact> contacts;
		void Init()
		{
			contacts.resize(100);
		}
	};
}
