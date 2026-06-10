// GeoHelpersTests.cpp
// Run standalone or integrate with your build.
// #define CACTI_ASSERT_THROWS to get exceptions instead of abort on failure.

#include <cassert>
#include <cmath>
#include <vector>
#include "Math/GeoHelpers.h"

namespace Cacti {

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
        T3_interior_points_excluded();
        T4_all_inputs_inside_hull();
        T5_outward_facing_normals();
        T6_duplicate_points_no_crash();
        T8_distance_sign_convention();
        T9_is_inside_tetrahedron();
        T10_coplanar_redundant_points();
        // All assertions passed if we reach here
    }

} // namespace Cacti