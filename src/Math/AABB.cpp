#include "AABB.h"

namespace Cacti
{
	AABB::AABB()
	{
		Clear();
	}
	AABB::~AABB()
	{

	}
	const AABB& AABB::operator=(const AABB& aabb)
	{
		min = aabb.min;
		max = aabb.max;
		return *this;
	}
	void AABB::ExpandToContainPoint(const Vec3& p)
	{
		if (p.x < min.x)
		{
			min.x = p.x;
		}
		if (p.y < min.y)
		{
			min.y = p.y;
		}
		if (p.z < min.z)
		{
			min.z = p.z;
		}

		if (p.x > max.x)
		{
			max.x = p.x;
		}
		if (p.y > max.y)
		{
			max.y = p.y;
		}
		if (p.z > max.z)
		{
			max.z = p.z;
		}
	}
	bool AABB::DoesIntersect(const AABB& aabb)
	{
		if (this->max.x <= aabb.min.x)
		{
			return false;
		}
		if (this->max.y <= aabb.min.y)
		{
			return false;
		}
		if (this->max.z <= aabb.min.z)
		{
			return false;
		}

		if (aabb.max.x <= this->min.x)
		{
			return false;
		}
		if (aabb.max.y <= this->min.y)
		{
			return false;
		}
		if (aabb.max.z <= this->min.z)
		{
			return false;
		}

		return true;
	}
	void AABB::ExpandToContainPoints(const Vec3* pts, const int num)
	{
		for (int i = 0; i < num; i++)
		{
			ExpandToContainPoint(pts[i]);
		}
	}
	void AABB::ExpandByAABB(const AABB& aabb)
	{
		ExpandToContainPoint(aabb.min);
		ExpandToContainPoint(aabb.max);
	}
}