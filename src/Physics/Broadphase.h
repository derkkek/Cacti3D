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

		Broadphase();
		~Broadphase() = default;

		static Vec3 projectionAxis;

		static std::vector<Projection1D> projectedAABBS;
		static std::vector<CollisionPair> collisionPairs;

		static std::vector<CollisionPair>& SweepAndPrune(std::vector<Body>& bodies);
	private:
		static void ProjectAABSofBodiesOnToTheAxis(std::vector<Body>& bodies);
		static void SortProjections();
		static void CreateCollisionPairs(std::vector<Body>& bodies);

	};

}
