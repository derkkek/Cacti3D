#include "Broadphase.h"
#include <algorithm>
namespace Cacti
{
	Broadphase::Broadphase(int bodiesSize)
		:projectionAxis(Vec3(1,1,1).Normalized())
	{
		projectedAABBS.reserve(bodiesSize);
		collisionPairs.reserve(bodiesSize * 4);
	}
	std::vector<Broadphase::CollisionPair>& Broadphase::SweepAndPrune(std::vector<Body>& bodies)
	{
		ProjectAABSofBodiesOnToTheAxis(bodies);
		SortProjections();
		CreateCollisionPairs(bodies);
		return this->collisionPairs;
	}
	void Broadphase::ProjectAABSofBodiesOnToTheAxis(std::vector<Body>& bodies)
	{
		projectedAABBS.resize(bodies.size());

		for (int i = 0; i < bodies.size(); i++)
		{
			Body &body = bodies[i];

			AABB b = body.shape->GetAABBWorldSpace(body.position, body.orientation);

			Vec3 C = b.GetCenter();

			float projectedCenterOnToAxis = C.Dot(projectionAxis);

			float projectionRadius = b.WidthX() * 0.5f * fabsf(projectionAxis.x) + b.WidthY() * 0.5f * fabsf(projectionAxis.y) + b.WidthZ() * 0.5f * fabsf(projectionAxis.z);

			projectedAABBS[i] = { projectedCenterOnToAxis - projectionRadius, projectedCenterOnToAxis + projectionRadius, i };
		}
	}
	void Broadphase::SortProjections()
	{
		for (int i = 1; i < (int)projectedAABBS.size(); i++)
		{
			Projection1D key = projectedAABBS[i];
			int j = i - 1;
			while (j >= 0 && projectedAABBS[j].left > key.left)
			{
				projectedAABBS[j + 1] = projectedAABBS[j];
				j--;
			}
			projectedAABBS[j + 1] = key;
		}
	}

	void Broadphase::CreateCollisionPairs(std::vector<Body>& bodies)
	{
		collisionPairs.clear();
		for (int i = 0; i < projectedAABBS.size() - 1; i++)
		{
			for (int j = i + 1; j < projectedAABBS.size(); j++)
			{
				if (projectedAABBS[j].left > projectedAABBS[i].right)
				{
					break;
				}
				collisionPairs.push_back({
					&bodies[projectedAABBS[i].bodyIndex],
					&bodies[projectedAABBS[j].bodyIndex]
					});
			}
		}
	}

}