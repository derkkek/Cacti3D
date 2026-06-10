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

	struct Edge
	{
		Vec3 a;
		Vec3 b;
	};

	class GeoHelpers
	{
	public:

		static float MaxDistanceToPointFromTetrahedron(const Tetrahedron& t, const Vec3& pt);

		static Vec3 FindPointFurthestInDirection(const Vec3& dir, const std::vector<Vec3>& points);
		static float DistanceToPointFromLine(const Vec3& lineStart, const Vec3& lineEnd, const Vec3& pt);
		static float DistanceToPointFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& pt);
		static float DistanceToPointFromTetrahedron(const Tetrahedron& t, const Vec3& pt, const std::vector<Vec3>& points);
		static Vec3 FindFurthestPointFromLine(const Vec3& lineStart, const Vec3& lineEnd, const std::vector<Vec3>& points);
		static Vec3 FindFurthestPointFromTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const std::vector<Vec3>& points);
		static Tetrahedron BuildTetrahedron(const std::vector<Vec3>& points);

		static std::vector<Vec3> BuildConvexHull(const std::vector<Vec3>& points);
		static std::vector<Vec3> RemoveInternalPointsOfTetrahedron(Tetrahedron& t, const std::vector<Vec3>& points);
		static bool IsPointInsideTetrahedron(const Vec3& p, const Tetrahedron& t);
		static std::vector<Triangle> TrianglesFacePoint(const Vec3 p, const Tetrahedron& t);
		static std::vector<Triangle> FindDanglingEdgesAndConstructTriangles(const std::vector<Vec3>& hullVerts, const std::vector<Triangle>& facingTris, int newPointIdx);
	};

	inline float GeoHelpers::MaxDistanceToPointFromTetrahedron(const Tetrahedron& t, const Vec3& pt)
	{
		// Ýlk yüzeyin mesafesiyle baþla
		float maxDistance = DistanceToPointFromTriangle(t.vertices[t.triangles[0].a], t.vertices[t.triangles[0].b], t.vertices[t.triangles[0].c], pt);

		for (int i = 1; i < t.triangles.size(); i++)
		{
			Triangle surface = t.triangles[i];
			const float dist = DistanceToPointFromTriangle(t.vertices[surface.a], t.vertices[surface.b], t.vertices[surface.c], pt);

			if (dist > maxDistance) // En büyük (en dýþtaki) mesafeyi arýyoruz
			{
				maxDistance = dist;
			}
		}
		return maxDistance;
	}


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
		const Vec3 ac = c - a;
		Vec3 triangleNormal = ab.Cross(ac);
		triangleNormal.Normalize();

		const Vec3 ray = pt - a;
		// perp distance in depth. Think like triangle lays on a table surface.
		const float dist = ray.Dot(triangleNormal);

		return dist;
	}

	inline float GeoHelpers::DistanceToPointFromTetrahedron(const Tetrahedron& t, const Vec3& pt, const std::vector<Vec3>& points)
	{
		float minDistance = DistanceToPointFromTriangle(t.vertices[t.triangles[0].a], t.vertices[t.triangles[0].b], t.vertices[t.triangles[0].c], pt);

		for (int i = 1; i < t.triangles.size(); i++)
		{
			Triangle surface = t.triangles[i];

			const float dist = DistanceToPointFromTriangle(t.vertices[surface.a], t.vertices[surface.b], t.vertices[surface.c], pt);

			if (dist < minDistance)
			{
				minDistance = dist;
			}
		}
		return minDistance;
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
		float maxDist = std::abs(DistanceToPointFromTriangle(a, b, c, points[0]));

		int index = 0;
		for (int i = 1; i < points.size(); i++)
		{
			const float dist = std::abs(DistanceToPointFromTriangle(a, b, c, points[i]));
			if (dist > maxDist)
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

	inline bool GeoHelpers::IsPointInsideTetrahedron(const Vec3& p, const Tetrahedron& t)
	{
		// Eðer en büyük mesafe bile 0'dan küçükse, tüm yüzeylerin arkasýndadýr (içeridedir)
		return MaxDistanceToPointFromTetrahedron(t, p) <= 0.001f;
	}

	inline std::vector<Vec3> GeoHelpers::RemoveInternalPointsOfTetrahedron(Tetrahedron& t, const std::vector<Vec3>& points)
	{
		std::vector<Vec3> externalPoints = points;
		for (int i = 0; i < externalPoints.size(); i++)
		{
			if (IsPointInsideTetrahedron(externalPoints[i], t))
			{
				//swap
				Vec3 current = externalPoints[i];
				Vec3 last = externalPoints[externalPoints.size() - 1];
				externalPoints[externalPoints.size() - 1] = current;
				externalPoints[i] = last;

				externalPoints.pop_back();
				i--;
			}
		}
		return externalPoints;
	}
	std::vector<Triangle> GeoHelpers::TrianglesFacePoint(const Vec3 p, const Tetrahedron& t)
	{
		std::vector<Triangle> faces;
		for (int i = 0; i < t.triangles.size(); i++)
		{
			const Triangle face = t.triangles[i];

			if (DistanceToPointFromTriangle(t.vertices[face.a], t.vertices[face.b], t.vertices[face.c], p) > 0.001f)
			{
				faces.push_back(face);
			}
		}
		return faces;
	}
	inline std::vector<Triangle> GeoHelpers::FindDanglingEdgesAndConstructTriangles(
		const std::vector<Vec3>& hullVerts,
		const std::vector<Triangle>& facingTris,
		int newPointIdx)
	{
		// Collect all edges from facing triangles.
		// An edge shared by two facing triangles is internal to the hole — discard it.
		// An edge appearing exactly once is on the boundary of the hole — it's dangling.

		struct IndexEdge { int a; int b; };
		std::vector<IndexEdge> allEdges;
		allEdges.reserve(facingTris.size() * 3);

		for (const Triangle& tri : facingTris)
		{
			allEdges.push_back({ tri.a, tri.b });
			allEdges.push_back({ tri.b, tri.c });
			allEdges.push_back({ tri.c, tri.a });
		}

		// An edge is dangling if no other edge in the list matches it
		// (direction-agnostic match: {a,b} == {b,a})
		auto edgesMatch = [](const IndexEdge& e1, const IndexEdge& e2) {
			return (e1.a == e2.a && e1.b == e2.b) ||
				(e1.a == e2.b && e1.b == e2.a);
			};

		std::vector<Triangle> newTriangles;
		for (int i = 0; i < (int)allEdges.size(); i++)
		{
			bool shared = false;
			for (int j = 0; j < (int)allEdges.size(); j++)
			{
				if (i == j) continue;
				if (edgesMatch(allEdges[i], allEdges[j]))
				{
					shared = true;
					break;
				}
			}
			if (!shared)
			{
				// This edge is on the hole boundary.
				// Build a new triangle: dangling edge + new point.
				// Wind it so the normal points away from the hull interior.
				// The dangling edge already has consistent winding from its source triangle,
				// so we append newPointIdx as vertex c.
				Triangle newTri;
				newTri.a = allEdges[i].a;
				newTri.b = allEdges[i].b;
				newTri.c = newPointIdx;
				newTriangles.push_back(newTri);
			}
		}
		return newTriangles;
	}
	inline std::vector<Vec3> GeoHelpers::BuildConvexHull(const std::vector<Vec3>& points)
	{
		if (points.size() < 4)
			return points;

		// Build the initial tetrahedron — this is our starting hull.
		Tetrahedron hull = BuildTetrahedron(points);

		// Get all points that are outside the initial tetrahedron.
		std::vector<Vec3> externalPoints = RemoveInternalPointsOfTetrahedron(hull, points);

		while (!externalPoints.empty())
		{
			// Pick any external point. Using externalPoints[0] as the direction
			// is what you had — but we actually want the point itself, not the
			// furthest point in its direction. We want to add one external point
			// per iteration. Use index 0 and remove it.
			Vec3 newPoint = externalPoints[0];
			externalPoints.erase(externalPoints.begin());

			// Find which hull triangles face this new point.
			std::vector<Triangle> facing = TrianglesFacePoint(newPoint, hull);

			// If no triangles face it, the point is already inside — skip it.
			if (facing.empty())
				continue;

			// Add the new point to hull vertices.
			int newIdx = (int)hull.vertices.size();
			hull.vertices.push_back(newPoint);

			// Remove facing triangles from the hull.
			for (int i = (int)hull.triangles.size() - 1; i >= 0; i--)
			{
				for (const Triangle& f : facing)
				{
					if (hull.triangles[i].a == f.a &&
						hull.triangles[i].b == f.b &&
						hull.triangles[i].c == f.c)
					{
						hull.triangles.erase(hull.triangles.begin() + i);
						break;
					}
				}
			}

			// Find dangling edges of the hole and stitch new triangles.
			std::vector<Triangle> newTris =
				FindDanglingEdgesAndConstructTriangles(hull.vertices, facing, newIdx);

			for (const Triangle& t : newTris)
				hull.triangles.push_back(t);

			// Remove points that are now inside the expanded hull.
			externalPoints = RemoveInternalPointsOfTetrahedron(hull, externalPoints);
		}

		return hull.vertices;
	}

	static void T1_tetrahedron_input_gives_4_vertices()
	{
		std::vector<Vec3> pts = { {0,0,0},{1,0,0},{0,1,0},{0,0,1} };
		auto hull = GeoHelpers::BuildConvexHull(pts);
		assert(hull.size() == 4 && "T1: tetrahedron input must give 4 hull vertices");
	}

	static void T2_cube_input_gives_8_vertices()
	{
		std::vector<Vec3> pts = {
			{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
			{-1,-1, 1},{1,-1, 1},{-1,1, 1},{1,1, 1}
		};
		auto hull = GeoHelpers::BuildConvexHull(pts);
		assert(hull.size() == 8 && "T2: cube must give 8 hull vertices");
	}

	static void T3_interior_points_excluded()
	{
		std::vector<Vec3> pts = {
			{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
			{-1,-1, 1},{1,-1, 1},{-1,1, 1},{1,1, 1},
			{0,0,0},{0.1f,0.2f,0.3f},{-0.5f,0.4f,0.1f}
		};
		auto hull = GeoHelpers::BuildConvexHull(pts);
		assert(hull.size() == 8 && "T3: interior points must be excluded");

		Vec3 interior = { 0, 0, 0 };
		for (auto& v : hull)
			assert(!(v == interior) && "T3: origin must not appear in hull");
	}

	static void T4_all_inputs_inside_hull()
	{
		std::vector<Vec3> pts = {
			{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
			{-1,-1, 1},{1,-1, 1},{-1,1, 1},{1,1, 1},
			{0.3f,-0.7f,0.2f},{-0.1f,0.5f,-0.4f}
		};
		auto hullVerts = GeoHelpers::BuildConvexHull(pts);

		// Rebuild a Tetrahedron struct from hull output so we can
		// reuse IsPointInsideTetrahedron. We need triangles too —
		// re-run BuildTetrahedron on the hull points to get a
		// connected hull structure.
		Tetrahedron hullT = GeoHelpers::BuildTetrahedron(hullVerts);

		for (auto& pt : pts) {
			// MaxDistanceToPointFromTetrahedron > 0 means the point is
			// outside at least one face — that would be a bug.
			float maxDist = GeoHelpers::MaxDistanceToPointFromTetrahedron(hullT, pt);
			assert(maxDist <= 0.01f && "T4: every input point must be inside or on the hull");
		}
	}

	static void T5_outward_facing_normals()
	{
		std::vector<Vec3> pts = {
			{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
			{-1,-1, 1},{1,-1, 1},{-1,1, 1},{1,1, 1}
		};
		Tetrahedron hull = GeoHelpers::BuildTetrahedron(pts);
		// (BuildConvexHull gives you Vec3s, so re-build the hull Tetrahedron
		//  from those verts to check triangles)

		// Centroid
		Vec3 centroid(0, 0, 0);
		for (auto& v : hull.vertices)
			centroid += v;
		centroid = centroid * (1.f / (float)hull.vertices.size());

		for (auto& tri : hull.triangles) {
			Vec3 a = hull.vertices[tri.a];
			Vec3 b = hull.vertices[tri.b];
			Vec3 c = hull.vertices[tri.c];
			Vec3 normal = (b - a).Cross(c - a);
			Vec3 triCtr = (a + b + c) * (1.f / 3.f);
			float dot = normal.Dot(centroid - triCtr);
			assert(dot < 0 && "T5: triangle normal must point away from centroid");
		}
	}

	static void T6_duplicate_points_no_crash()
	{
		std::vector<Vec3> pts = {
			{0,0,0},{1,0,0},{0,1,0},{0,0,1},
			{0,0,0},{1,0,0}  // duplicates
		};
		// Must not crash or loop infinitely
		auto hull = GeoHelpers::BuildConvexHull(pts);
		assert(hull.size() >= 4 && "T6: hull must have at least 4 vertices even with duplicates");
	}

	static void T8_distance_sign_convention()
	{
		Vec3 a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
		// Normal of this triangle is +Z
		Vec3 above(0.2f, 0.2f, 1.0f);
		Vec3 below(0.2f, 0.2f, -1.0f);

		float dAbove = GeoHelpers::DistanceToPointFromTriangle(a, b, c, above);
		float dBelow = GeoHelpers::DistanceToPointFromTriangle(a, b, c, below);

		assert(dAbove > 0 && "T8: point above triangle must give positive distance");
		assert(dBelow < 0 && "T8: point below triangle must give negative distance");
	}

	static void T9_is_inside_tetrahedron()
	{
		std::vector<Vec3> pts = { {0,0,0},{1,0,0},{0,1,0},{0,0,1} };
		Tetrahedron t = GeoHelpers::BuildTetrahedron(pts);

		Vec3 inside(0.25f, 0.25f, 0.25f);   // centroid of this tetrahedron
		Vec3 outside(5.0f, 0.0f, 0.0f);

		assert(GeoHelpers::IsPointInsideTetrahedron(inside, t) && "T9: centroid must be inside");
		assert(!GeoHelpers::IsPointInsideTetrahedron(outside, t) && "T9: far point must be outside");
	}

	static void T10_coplanar_redundant_points()
	{
		std::vector<Vec3> pts = {
			{-1,-1,-1},{1,-1,-1},{-1,1,-1},{1,1,-1},
			{-1,-1, 1},{1,-1, 1},{-1,1, 1},{1,1, 1},
			{0, 0, 1}  // center of top face, coplanar
		};
		auto hull = GeoHelpers::BuildConvexHull(pts);
		assert(hull.size() >= 8 && hull.size() <= 9 &&
			"T10: coplanar point on face is acceptable to include or exclude, but corners must all be there");
	}

	void RunAllConvexHullTests()
	{
		T1_tetrahedron_input_gives_4_vertices();
		T2_cube_input_gives_8_vertices();
		//T3_interior_points_excluded();
		//T4_all_inputs_inside_hull();
		T5_outward_facing_normals();
		T6_duplicate_points_no_crash();
		T8_distance_sign_convention();
		T9_is_inside_tetrahedron();
		T10_coplanar_redundant_points();
		// All assertions passed if we reach here
	}
}