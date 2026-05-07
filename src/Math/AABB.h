#pragma once
#include "Math/Vector.h"

namespace Cacti
{
	class AABB
	{
	public:
		AABB();
		~AABB();
		const AABB& operator=(const AABB& aabb);

		Vec3 min;
		Vec3 max;

		/* Add(p) in some implementations*/

		inline void Clear() { min = 1e6; max = -1e6; }

		bool DoesIntersect(const AABB& aabb);

		void ExpandToContainPoint(const Vec3& p);

		void ExpandToContainPoints(const Vec3* pts, const int num);

		void ExpandByAABB(const AABB& aabb);

		inline float WidthX() const { return max.x - min.x; }
		inline float WidthY() const { return max.y - min.y; }
		inline float WidthZ() const { return max.z - min.z; }

		inline Vec3 GetCenter() { return Vec3((min.x + max.x) + 2, (min.y + max.y) + 2, (min.z + max.z) + 2); }
	private:

	};
}