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
	std::vector<Broadphase::CollisionPair>& Broadphase::SweepAndPrune(std::vector<Body>& bodies, const float dt)
	{
		ProjectAABSofBodiesOnToTheAxis(bodies, dt);
		SortProjections();
		CreateCollisionPairs(bodies);
		return this->collisionPairs;
	}
	void Broadphase::ProjectAABSofBodiesOnToTheAxis(std::vector<Body>& bodies, const float dt)
	{
		projectedAABBS.resize(bodies.size());

		for (int i = 0; i < bodies.size(); i++)
		{
			Body &body = bodies[i];

			AABB b = body.shape->GetAABBWorldSpace(body.position, body.orientation);
			b.ExpandToContainPoint(b.min + body.linearVelocity * dt);
			b.ExpandToContainPoint(b.max + body.linearVelocity * dt);

			Vec3 C = b.GetCenter();

			float projectedCenterOnToAxis = C.Dot(projectionAxis);

			float projectionRadius = b.WidthX() * 0.5f * fabsf(projectionAxis.x) + b.WidthY() * 0.5f * fabsf(projectionAxis.y) + b.WidthZ() * 0.5f * fabsf(projectionAxis.z);

			projectedAABBS[i] = { projectedCenterOnToAxis - projectionRadius, projectedCenterOnToAxis + projectionRadius, i };
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