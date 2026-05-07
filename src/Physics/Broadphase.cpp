#include "Broadphase.h"
#include <algorithm>
namespace Cacti
{
	Vec3 Broadphase::projectionAxis;
	std::vector<Broadphase::Projection1D> Broadphase::projectedAABBS;
	std::vector<Broadphase::CollisionPair> Broadphase::collisionPairs;
	
	Broadphase::Broadphase()
	{
		projectionAxis = Vec3(1, 1, 1);
		projectionAxis.Normalize();
	}
	std::vector<Broadphase::CollisionPair>& Broadphase::SweepAndPrune(std::vector<Body>& bodies)
	{
		ProjectAABSofBodiesOnToTheAxis(bodies);
		SortProjections();
		CreateCollisionPairs(bodies);
		return collisionPairs;
	}
	void Broadphase::ProjectAABSofBodiesOnToTheAxis(std::vector<Body>& bodies)
	{
		projectedAABBS.clear();
		projectedAABBS.reserve(bodies.size());

		for (int i = 0; i < bodies.size(); i++)
		{
			Body &body = bodies[i];

			AABB b = body.shape->GetAABBWorldSpace(body.position, body.orientation);

			Vec3 C = b.GetCenter();

			float projectedCenterOnToAxis = C.Dot(projectionAxis);

			float projectionRadius = b.WidthX() * 0.5f * fabsf(projectionAxis.x) + b.WidthY() * 0.5f * fabsf(projectionAxis.y) + b.WidthZ() * 0.5f * fabsf(projectionAxis.z);

			Projection1D projection{ projectedCenterOnToAxis - projectionRadius, projectedCenterOnToAxis + projectionRadius, i };

			projectedAABBS.push_back(projection);
		}
	}
	void Broadphase::SortProjections()
	{
		std::sort(projectedAABBS.begin(), projectedAABBS.end(), [](const Projection1D& a, const Projection1D& b)
			{
				if (a.left != b.left)
				{
					return a.left < b.left;
				}

				return a.right < b.right;
			});
	}

	void Broadphase::CreateCollisionPairs(std::vector<Body>& bodies)
	{
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