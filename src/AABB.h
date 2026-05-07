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
		void ExpandToContainPoint(const Vec3& p);

		inline void Clear() { min = 1e6; max = -1e6; }

		bool DoesIntersect(const AABB& aabb);

		void ExpandToContainPoints(const Vec3* pts, const int num);

		void ExpandByAABB(const AABB& aabb);

		inline float WidthX() const { max.x - min.x; }
		inline float WidthY() const { max.y - min.y; }
		inline float WidthZ() const { max.z - min.z; }
	private:

	};
}