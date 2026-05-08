#pragma once
#include "../Math/Vector.h"
#include "../Math/AABB.h"
#include "../Physics/Body.h"
#include <vector>
namespace Cacti
{
	class Broadphase
	{
		struct Projection1D
		{
			float left;
			float right;
			int bodyIndex;
		};


	public:
		struct CollisionPair
		{
			Body* a;
			Body* b;
		};

		Broadphase(int bodiesSize);
		~Broadphase() = default;

		Vec3 projectionAxis;

		std::vector<Projection1D> projectedAABBS;
		std::vector<CollisionPair> collisionPairs;

		std::vector<CollisionPair>& SweepAndPrune(std::vector<Body>& bodies);
	private:
		void ProjectAABSofBodiesOnToTheAxis(std::vector<Body>& bodies);
		void SortProjections();
		void CreateCollisionPairs(std::vector<Body>& bodies);

	};

}
