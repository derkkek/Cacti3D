#pragma once
#include "Vector.h"
#include <vector>
namespace Cacti
{
	struct Triangle
	{
		int a;
		int b;
		int c;
	};

	struct Tetrahedron
	{
		std::vector<Triangle> triangles;
		std::vector<Vec3> vertices;
	};

	class GeoHelpers
	{
	public:

		static Vec3 FindPointFurthestInDirection(const Vec3& dir, const std::vector<Vec3>& points);
		static float DistanceToPointFromLine(const Vec3& lineStart, const Vec3& lineEnd, const Vec3& pt);
		static float DistanceToPointFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt);
		static Vec3 FindFurthestPointFromLine(const Vec3& lineStart, const Vec3& lineEnd, const std::vector<Vec3>& points);
		static Vec3 FindFurthestPointFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const std::vector<Vec3>& points);
		static Tetrahedron BuildTetrahedron(const std::vector<Vec3>& points);
		static std::vector<Vec3> BuildConvexHull(const std::vector<Vec3>& points);
		static std::vector<Vec3> RemoveInternalPointsOfTetrahedron(Tetrahedron& t);
	};


	inline Vec3 GeoHelpers::FindPointFurthestInDirection(const Vec3& dir, const std::vector<Vec3>& points)
	{
		int index = 0;
		float maxDist = points[0].Dot(dir);
		for (int i = 1; i < points.size(); i++)
		{
			if (points[i].Dot(dir) > maxDist)
			{
				maxDist = points[i].Dot(dir);
				index = i;
			}
		}
		return points[index];
	}

	inline float GeoHelpers::DistanceToPointFromLine(const Vec3& lineStart, const Vec3& lineEnd, const Vec3& pt)
	{
		Vec3 ab = lineEnd - lineStart;
		ab.Normalize(); // we need to normalize because of the dot product formula. If we don't it scales the pt projection on ab by magn of ab

		Vec3 lineStartToPoint = pt - lineStart;

		float abProjectionOf_lineStartToPoint = lineStartToPoint.Dot(ab);

		Vec3 lineStartToJustUnderThePoint = ab * abProjectionOf_lineStartToPoint;

		float perpDistanceFromPointToTheAB = (pt - lineStartToJustUnderThePoint).GetMagnitude();

		return perpDistanceFromPointToTheAB;
	}

	inline float GeoHelpers::DistanceToPointFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt)
	{
		const Vec3 ab = b - a;
		const Vec3 bc = b - c;
		Vec3 triangleNormal = ab.Cross(bc);
		triangleNormal.Normalize();

		// perp distance in depth. Think like triangle lays on a table surface.
		const float dist = pt.Dot(triangleNormal);

		return dist;
	}

	/*c++ native, consider changing vector input by array pointer.*/
	inline Vec3 GeoHelpers::FindFurthestPointFromLine(const Vec3& lineStart, const Vec3& lineEnd, const std::vector<Vec3>& points)
	{
		float max = DistanceToPointFromLine(lineStart, lineEnd, points[0]);
		int index = 0;
		for (int i = 1; i < points.size(); i++)
		{
			const float pDistanceToLine = DistanceToPointFromLine(lineStart, lineEnd, points[i]);
			if (pDistanceToLine > max)
			{
				max = pDistanceToLine;
				index = i;
			}
		}
		return points[index];
	}

	inline Vec3 GeoHelpers::FindFurthestPointFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const std::vector<Vec3>& points)
	{
		float maxDist = DistanceToPointFromTriangle(a, b, c, points[0]);

		int index = 0;
		for (int i = i; i < points.size(); i++)
		{
			const float dist = DistanceToPointFromTriangle(a, b, c, points[i]);
			if (dist * dist > maxDist * maxDist)
			{
				maxDist = dist;
				index = i;
			}
		}

		return points[index];
	}

	inline Tetrahedron GeoHelpers::BuildTetrahedron(const std::vector<Vec3>& points)
	{
		Vec3 initialP = points[0];

		for (int i = 1; i < points.size(); i++)
		{
			if (points[i].x < initialP.x)
			{
				initialP = points[i];
			}
		}

		Vec3 secondP = points[0];

		float maxDist = (initialP - secondP).GetLengthSqr();

		for (int i = 1; i < points.size(); i++)
		{
			const Vec3 ab = initialP - points[i];
			if (ab.GetLengthSqr() > maxDist)
			{
				secondP = points[i];
			}
		}


		Vec3 thirdP = FindFurthestPointFromLine(secondP, initialP, points);

		Vec3 fourthP = FindFurthestPointFromTriangle(initialP, secondP, thirdP, points);

		if (DistanceToPointFromTriangle(initialP, secondP, thirdP, fourthP) > 0)
		{
			std::swap(initialP, secondP);
		}

		Tetrahedron t;
		t.vertices = { initialP, secondP, thirdP, fourthP };
		Triangle t1 = { 0, 1, 2 };
		Triangle t2 = { 0, 2, 3 };
		Triangle t3 = { 2, 1, 3 };
		Triangle t4 = { 1, 0, 3 };
		t.triangles = { t1, t2, t3, t4 };

		return t;
	}
	inline std::vector<Vec3> GeoHelpers::BuildConvexHull(const std::vector<Vec3>& points)
	{
		Tetrahedron tetra = BuildTetrahedron(points);
		std::vector<Vec3> tetraSurfacePoints = RemoveInternalPointsOfTetrahedron(tetra);
		return std::vector<Vec3>();
	}
	inline std::vector<Vec3> GeoHelpers::RemoveInternalPointsOfTetrahedron(Tetrahedron& t)
	{
		return std::vector<Vec3>();
	}
}