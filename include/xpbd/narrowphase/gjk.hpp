#ifndef XPBD_NARROWPHASE_GJK_HPP
#define XPBD_NARROWPHASE_GJK_HPP

// Shared convex-convex narrowphase cell (DESIGN.md 5, narrowphase.hpp matrix).
//
// GJK + EPA on *support functions*. A support function support(d) returns the
// farthest point of a convex set along direction d; that is all GJK/EPA need, so
// one implementation covers every convex primitive. Each shape is modelled as a
// core skeleton (a point, a segment, a vertex set) plus a rounding radius: the
// full shape is the Minkowski sum of the core with a ball of that radius. GJK
// runs on the cores; the radii are folded in afterwards. So a sphere is a point
// with radius, a capsule a segment with radius, and a box / tetra / hull a
// vertex set with radius 0. GJK finds the closest distance between cores (cheap,
// allocation-free); only when the cores actually overlap do we fall back to EPA
// to recover the penetration depth and normal.
//
// Output is the canonical Contact (contact.hpp): unit normal A -> B, world
// contact point, penetration depth (>= 0 when touching).

#include "xpbd/math.hpp"
#include "xpbd/narrowphase/contact.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace xpbd::narrowphase {

// --- Convex shape adapters --------------------------------------------------
// A convex type for gjkEpaContact must provide:
//   Vec3  support(const Vec3& dir) const;  // farthest *core* point along dir
//   float radius() const;                  // rounding radius (0 for polytopes)

// Sphere: a single core point rounded by its radius.
struct GjkSphere {
    Vec3 center{};
    float r = 0.0f;
    Vec3 support(const Vec3&) const { return center; }
    float radius() const { return r; }
};

// Capsule: a core segment [a, b] rounded by its radius.
struct GjkCapsule {
    Vec3 a{};
    Vec3 b{};
    float r = 0.0f;
    Vec3 support(const Vec3& dir) const { return dot(a, dir) >= dot(b, dir) ? a : b; }
    float radius() const { return r; }
};

// Convex hull of an arbitrary world-space point set (box = 8 verts, tetra = 4,
// triangle = 3, ...). Linear-scan support; fine for the small vertex counts the
// narrowphase deals with. Rounding radius lets a hull be inflated (rounded box).
struct GjkHull {
    const Vec3* points = nullptr;
    int count = 0;
    float r = 0.0f;

    GjkHull() = default;
    GjkHull(const Vec3* pts, int n, float radius = 0.0f) : points(pts), count(n), r(radius) {}

    Vec3 support(const Vec3& dir) const {
        int best = 0;
        float bestDot = dot(points[0], dir);
        for (int i = 1; i < count; ++i) {
            const float d = dot(points[i], dir);
            if (d > bestDot) {
                bestDot = d;
                best = i;
            }
        }
        return points[best];
    }
    float radius() const { return r; }
};

// Axis-aligned box given by min/max corners (a convenience hull without storage).
struct GjkAabb {
    Vec3 lo{};
    Vec3 hi{};
    float r = 0.0f;
    Vec3 support(const Vec3& dir) const {
        return {dir.x >= 0.0f ? hi.x : lo.x,
                dir.y >= 0.0f ? hi.y : lo.y,
                dir.z >= 0.0f ? hi.z : lo.z};
    }
    float radius() const { return r; }
};

namespace detail {

// A vertex of the Minkowski difference A (-) B, keeping the witness points on
// each core so the contact point can be recovered by barycentric interpolation.
struct SupportVertex {
    Vec3 v{};   // support_A(d) - support_B(-d), the Minkowski-difference point
    Vec3 a{};   // witness on core A
    Vec3 b{};   // witness on core B
};

template <class ShapeA, class ShapeB>
SupportVertex minkowskiSupport(const ShapeA& a, const ShapeB& b, const Vec3& dir) {
    SupportVertex s;
    s.a = a.support(dir);
    s.b = b.support(-dir);
    s.v = s.a - s.b;
    return s;
}

// Closest point on segment [a, b] to the origin, returned as barycentric
// weights (wa, wb) with wa + wb == 1. Used by the GJK sub-distance routine.
inline void closestOnSegment(const Vec3& a, const Vec3& b, float& wa, float& wb) {
    const Vec3 ab = b - a;
    const float denom = dot(ab, ab);
    float t = denom > 1e-20f ? dot(Vec3{} - a, ab) / denom : 0.0f;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    wb = t;
    wa = 1.0f - t;
}

// Barycentric coordinates of the origin's projection onto triangle (a, b, c),
// clamped to the triangle. Returns weights summing to 1. Ericson, RTCD.
inline void closestOnTriangle(const Vec3& a, const Vec3& b, const Vec3& c,
                              float& wa, float& wb, float& wc) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = Vec3{} - a;
    const float d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) { wa = 1.0f; wb = 0.0f; wc = 0.0f; return; }
    const Vec3 bp = Vec3{} - b;
    const float d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) { wa = 0.0f; wb = 1.0f; wc = 0.0f; return; }
    const Vec3 cp = Vec3{} - c;
    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) { wa = 0.0f; wb = 0.0f; wc = 1.0f; return; }
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float t = d1 / (d1 - d3);
        wa = 1.0f - t; wb = t; wc = 0.0f; return;
    }
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float t = d2 / (d2 - d6);
        wa = 1.0f - t; wb = 0.0f; wc = t; return;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        wa = 0.0f; wb = 1.0f - t; wc = t; return;
    }
    const float denom = 1.0f / (va + vb + vc);
    wb = vb * denom;
    wc = vc * denom;
    wa = 1.0f - wb - wc;
}

}  // namespace detail

namespace detail {
struct SupportVertex;
}

// Result of the core GJK distance query (before radii are folded in).
struct GjkResult {
    bool intersecting = false;  // cores overlap -> distance undefined, run EPA
    float distance = 0.0f;      // distance between cores when separated
    Vec3 pointA{};              // closest point on core A
    Vec3 pointB{};              // closest point on core B
    Vec3 normal{};              // unit A -> B when separated (pointB - pointA)
    int iterations = 0;
    // The terminating simplex (1..4 vertices). On the intersecting path this
    // encloses (or touches) the origin and is the seed EPA expands from.
    std::array<detail::SupportVertex, 4> simplex{};
    int simplexCount = 0;
};

namespace detail {

// Reduce `simplex` (n in 1..4) to the sub-simplex closest to the origin, write
// the barycentric weights of the origin's projection into `bary`, and set `dir`
// to point from that closest feature toward the origin. Returns false only when
// a full tetrahedron encloses the origin (the cores overlap).
inline bool reduceSimplex(std::array<SupportVertex, 4>& simplex, int& n,
                          Vec3& dir, float bary[4]) {
    auto setDir = [&](const Vec3& closest) { dir = Vec3{} - closest; };

    if (n == 2) {
        float wa, wb;
        closestOnSegment(simplex[0].v, simplex[1].v, wa, wb);
        bary[0] = wa; bary[1] = wb;
        setDir(simplex[0].v * wa + simplex[1].v * wb);
        return true;
    }

    if (n == 3) {
        float wa, wb, wc;
        closestOnTriangle(simplex[0].v, simplex[1].v, simplex[2].v, wa, wb, wc);
        bary[0] = wa; bary[1] = wb; bary[2] = wc;
        setDir(simplex[0].v * wa + simplex[1].v * wb + simplex[2].v * wc);
        return true;
    }

    // n == 4: test the origin against the three faces sharing the newest vertex
    // (index 3). Keep the face whose plane the origin is outside of (pointing
    // away from the 4th vertex); if outside none, the origin is enclosed.
    const Vec3& A = simplex[0].v;
    const Vec3& B = simplex[1].v;
    const Vec3& C = simplex[2].v;
    const Vec3& D = simplex[3].v;

    struct Face { int i0, i1, i2, opp; };
    const Face faces[3] = {{0, 1, 3, 2}, {1, 2, 3, 0}, {2, 0, 3, 1}};
    const Vec3 verts[4] = {A, B, C, D};

    float bestDistSq = 3.4e38f;
    int bestFace = -1;
    float bestW[3] = {0, 0, 0};
    bool anyOutside = false;

    for (const Face& f : faces) {
        const Vec3& p0 = verts[f.i0];
        const Vec3& p1 = verts[f.i1];
        const Vec3& p2 = verts[f.i2];
        Vec3 nrm = cross(p1 - p0, p2 - p0);
        // Orient the normal away from the opposing vertex.
        if (dot(nrm, verts[f.opp] - p0) > 0.0f) {
            nrm = Vec3{} - nrm;
        }
        // Is the origin on the outer side of this face?
        if (dot(nrm, Vec3{} - p0) <= 0.0f) {
            continue;  // origin inside this face's halfspace
        }
        anyOutside = true;

        float wa, wb, wc;
        closestOnTriangle(p0, p1, p2, wa, wb, wc);
        const Vec3 closest = p0 * wa + p1 * wb + p2 * wc;
        const float distSq = lengthSq(closest);
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestFace = static_cast<int>(&f - faces);
            bestW[0] = wa; bestW[1] = wb; bestW[2] = wc;
        }
    }

    if (!anyOutside) {
        return false;  // origin enclosed -> overlap
    }

    const Face& f = faces[bestFace];
    const SupportVertex keep[3] = {simplex[f.i0], simplex[f.i1], simplex[f.i2]};
    simplex[0] = keep[0];
    simplex[1] = keep[1];
    simplex[2] = keep[2];
    n = 3;
    bary[0] = bestW[0]; bary[1] = bestW[1]; bary[2] = bestW[2]; bary[3] = 0.0f;
    setDir(simplex[0].v * bary[0] + simplex[1].v * bary[1] + simplex[2].v * bary[2]);
    return true;
}

// GJK distance between the two cores. Marches a simplex (1..4 vertices) of the
// Minkowski difference toward the origin; converges to the closest sub-simplex.
// If the origin is enclosed (tetrahedron contains it), the cores overlap and we
// report `intersecting` so the caller switches to EPA.
template <class ShapeA, class ShapeB>
GjkResult gjkDistance(const ShapeA& a, const ShapeB& b, int maxIterations = 32) {
    GjkResult result;

    std::array<SupportVertex, 4> simplex;
    int n = 0;

    Vec3 dir{1.0f, 0.0f, 0.0f};
    simplex[0] = minkowskiSupport(a, b, dir);
    n = 1;
    // Witnesses / closest point for the current (single-vertex) simplex. These
    // stay valid even if the loop converges on the first iteration, which is the
    // common case when a core is a single point (sphere) or short segment.
    result.pointA = simplex[0].a;
    result.pointB = simplex[0].b;
    Vec3 closestV = simplex[0].v;
    dir = Vec3{} - closestV;

    auto recordSimplex = [&] {
        result.simplexCount = n;
        for (int i = 0; i < n; ++i) result.simplex[static_cast<std::size_t>(i)] = simplex[i];
    };

    for (int iter = 0; iter < maxIterations; ++iter) {
        result.iterations = iter + 1;
        if (lengthSq(dir) < 1e-20f) {
            // Closest point is the origin: the cores touch/overlap -> EPA.
            result.intersecting = true;
            recordSimplex();
            return result;
        }

        const SupportVertex w = minkowskiSupport(a, b, dir);

        // No further progress toward the origin in the search direction => the
        // current closest feature is the answer.
        const float progress = dot(w.v, dir) - dot(closestV, dir);
        if (progress < 1e-8f) {
            break;
        }

        simplex[n++] = w;

        // Reduce the simplex to the sub-feature closest to the origin and set the
        // next search direction to the origin-ward normal of that feature.
        float bary[4] = {0, 0, 0, 0};
        if (!reduceSimplex(simplex, n, dir, bary)) {
            // Origin enclosed by a full tetrahedron -> overlap.
            result.intersecting = true;
            recordSimplex();
            return result;
        }

        // Recover witness points and the closest point from the barycentrics.
        Vec3 pA{}, pB{};
        for (int i = 0; i < n; ++i) {
            pA += simplex[i].a * bary[i];
            pB += simplex[i].b * bary[i];
        }
        result.pointA = pA;
        result.pointB = pB;
        closestV = pA - pB;  // == sum bary_i * simplex_i.v
    }

    recordSimplex();
    // normal is A -> B: from A's witness toward B's witness.
    const Vec3 ab = result.pointB - result.pointA;
    result.distance = std::sqrt(lengthSq(ab));
    result.normal = result.distance > 1e-9f ? ab / result.distance : Vec3{0.0f, 1.0f, 0.0f};
    return result;
}

// Barycentric weights of an arbitrary point `p` (assumed in-plane) on triangle
// (a, b, c). Used by EPA to recover witness points at the projected origin.
inline void closestOnTriangleAt(const Vec3& a, const Vec3& b, const Vec3& c,
                                const Vec3& p, float& wa, float& wb, float& wc) {
    const Vec3 v0 = b - a, v1 = c - a, v2 = p - a;
    const float d00 = dot(v0, v0), d01 = dot(v0, v1), d11 = dot(v1, v1);
    const float d20 = dot(v2, v0), d21 = dot(v2, v1);
    const float denom = d00 * d11 - d01 * d01;
    if (std::fabs(denom) < 1e-20f) { wa = 1.0f; wb = 0.0f; wc = 0.0f; return; }
    wb = (d11 * d20 - d01 * d21) / denom;
    wc = (d00 * d21 - d01 * d20) / denom;
    wa = 1.0f - wb - wc;
}

// Add a horizon edge, cancelling it if its reverse is already present (a shared
// edge between two visible faces is interior to the hole, not on the horizon).
inline void addHorizonEdge(std::vector<std::array<int, 2>>& horizon, int i, int j) {
    for (std::size_t k = 0; k < horizon.size(); ++k) {
        if (horizon[k][0] == j && horizon[k][1] == i) {
            horizon.erase(horizon.begin() + static_cast<std::ptrdiff_t>(k));
            return;
        }
    }
    horizon.push_back({i, j});
}

// --- EPA (Expanding Polytope Algorithm) -------------------------------------
// Recovers penetration depth + normal when the cores overlap. Builds a starting
// tetrahedron around the origin, then repeatedly pushes the closest face out
// along its normal with a fresh support point until it stops growing. The
// closest face at convergence gives the minimum translation that separates the
// cores; barycentrics on that face recover the witness points.

struct EpaResult {
    bool valid = false;
    float depth = 0.0f;   // penetration of the cores (>= 0)
    Vec3 normal{};        // unit A -> B (push B by +normal to separate)
    Vec3 pointA{};        // witness on core A
    Vec3 pointB{};        // witness on core B
};

struct EpaFace {
    int a, b, c;          // indices into the vertex list
    Vec3 normal{};        // unit outward normal
    float dist = 0.0f;    // signed distance from origin to the face plane
};

inline void epaFaceNormal(const std::vector<SupportVertex>& verts, EpaFace& f) {
    const Vec3& A = verts[f.a].v;
    const Vec3& B = verts[f.b].v;
    const Vec3& C = verts[f.c].v;
    Vec3 nrm = cross(B - A, C - A);
    const float len = std::sqrt(lengthSq(nrm));
    nrm = len > 1e-12f ? nrm / len : Vec3{0.0f, 1.0f, 0.0f};
    float d = dot(nrm, A);
    if (d < 0.0f) {  // ensure the normal points away from the origin
        nrm = Vec3{} - nrm;
        d = -d;
    }
    f.normal = nrm;
    f.dist = d;
}

template <class ShapeA, class ShapeB>
EpaResult epaPenetration(const ShapeA& a, const ShapeB& b, const GjkResult& seed,
                         int maxIterations = 48) {
    EpaResult out;

    // EPA expands the polytope GJK left enclosing the origin. Start from GJK's
    // terminating simplex and complete it to a non-degenerate tetrahedron; this
    // is far more robust than rebuilding a seed from scratch, because the GJK
    // simplex already brackets the origin.
    std::vector<SupportVertex> verts;
    verts.reserve(static_cast<std::size_t>(maxIterations) + 8);
    for (int i = 0; i < seed.simplexCount; ++i) {
        verts.push_back(seed.simplex[static_cast<std::size_t>(i)]);
    }

    auto addSupport = [&](const Vec3& dir) {
        verts.push_back(minkowskiSupport(a, b, dir));
        return verts.size() - 1;
    };

    // Grow to >= 2 distinct vertices.
    if (verts.empty()) addSupport(Vec3{1.0f, 0.0f, 0.0f});
    if (verts.size() < 2) {
        addSupport(Vec3{} - verts[0].v);
        if (lengthSq(verts[1].v - verts[0].v) < 1e-10f) {
            verts.pop_back();
            addSupport(Vec3{0.0f, 1.0f, 0.0f});
        }
    }

    // Grow to a non-degenerate triangle (third vertex off the segment line).
    if (verts.size() < 3) {
        const Vec3 ab = verts[1].v - verts[0].v;
        Vec3 axis = std::fabs(ab.x) <= std::fabs(ab.y) && std::fabs(ab.x) <= std::fabs(ab.z)
                        ? Vec3{1, 0, 0}
                        : (std::fabs(ab.y) <= std::fabs(ab.z) ? Vec3{0, 1, 0} : Vec3{0, 0, 1});
        Vec3 perp = cross(ab, axis);
        if (lengthSq(perp) < 1e-12f) perp = cross(ab, Vec3{0, 0, 1});
        addSupport(perp);
        if (lengthSq(cross(verts[1].v - verts[0].v, verts[2].v - verts[0].v)) < 1e-12f) {
            verts.pop_back();
            addSupport(Vec3{} - perp);
        }
    }

    // Grow to a non-degenerate tetrahedron (fourth vertex off the triangle plane).
    if (verts.size() < 4) {
        Vec3 triN = cross(verts[1].v - verts[0].v, verts[2].v - verts[0].v);
        addSupport(triN);
        if (std::fabs(dot(normalized(triN), verts[3].v - verts[0].v)) < 1e-6f) {
            verts.pop_back();
            addSupport(Vec3{} - triN);
        }
    }

    if (verts.size() < 4) {
        return out;  // could not build a tetrahedron (fully degenerate input)
    }

    // Orient the tetra so all faces wind outward (consistent CCW from outside).
    {
        const Vec3& A = verts[0].v;
        const Vec3& B = verts[1].v;
        const Vec3& C = verts[2].v;
        const Vec3& D = verts[3].v;
        if (dot(cross(B - A, C - A), D - A) > 0.0f) {
            std::swap(verts[1], verts[2]);  // flip winding
        }
    }

    std::vector<EpaFace> faces;
    faces.push_back({0, 1, 2, {}, 0.0f});
    faces.push_back({0, 3, 1, {}, 0.0f});
    faces.push_back({1, 3, 2, {}, 0.0f});
    faces.push_back({0, 2, 3, {}, 0.0f});
    for (EpaFace& f : faces) epaFaceNormal(verts, f);

    for (int iter = 0; iter < maxIterations; ++iter) {
        // Closest face to the origin.
        int closest = 0;
        for (int i = 1; i < static_cast<int>(faces.size()); ++i) {
            if (faces[i].dist < faces[closest].dist) closest = i;
        }
        const Vec3 dir = faces[closest].normal;
        const float faceDist = faces[closest].dist;

        const SupportVertex w = minkowskiSupport(a, b, dir);
        const float newDist = dot(w.v, dir);

        if (newDist - faceDist < 1e-4f) {
            // Converged: this face is on the Minkowski-difference boundary.
            const EpaFace& f = faces[static_cast<std::size_t>(closest)];
            float wa, wb, wc;
            const Vec3 proj = f.normal * f.dist;  // origin projected onto the face
            closestOnTriangleAt(verts[f.a].v, verts[f.b].v, verts[f.c].v, proj, wa, wb, wc);
            out.pointA = verts[f.a].a * wa + verts[f.b].a * wb + verts[f.c].a * wc;
            out.pointB = verts[f.a].b * wa + verts[f.b].b * wb + verts[f.c].b * wc;
            out.normal = f.normal;
            out.depth = f.dist;
            out.valid = true;
            return out;
        }

        // Remove every face the new point can "see"; collect the horizon edges.
        const int newIndex = static_cast<int>(verts.size());
        verts.push_back(w);

        std::vector<std::array<int, 2>> horizon;
        for (int i = static_cast<int>(faces.size()) - 1; i >= 0; --i) {
            const EpaFace& f = faces[static_cast<std::size_t>(i)];
            if (dot(f.normal, w.v - verts[f.a].v) > 1e-9f) {
                addHorizonEdge(horizon, f.a, f.b);
                addHorizonEdge(horizon, f.b, f.c);
                addHorizonEdge(horizon, f.c, f.a);
                faces.erase(faces.begin() + i);
            }
        }

        for (const std::array<int, 2>& e : horizon) {
            EpaFace nf{e[0], e[1], newIndex, {}, 0.0f};
            epaFaceNormal(verts, nf);
            faces.push_back(nf);
        }
        if (faces.empty()) break;
    }

    // Fallback: return the best face found if iteration ran out.
    if (!faces.empty()) {
        int closest = 0;
        for (int i = 1; i < static_cast<int>(faces.size()); ++i) {
            if (faces[i].dist < faces[closest].dist) closest = i;
        }
        const EpaFace& f = faces[static_cast<std::size_t>(closest)];
        float wa, wb, wc;
        closestOnTriangleAt(verts[f.a].v, verts[f.b].v, verts[f.c].v, f.normal * f.dist, wa, wb, wc);
        out.pointA = verts[f.a].a * wa + verts[f.b].a * wb + verts[f.c].a * wc;
        out.pointB = verts[f.a].b * wa + verts[f.b].b * wb + verts[f.c].b * wc;
        out.normal = f.normal;
        out.depth = f.dist;
        out.valid = true;
    }
    return out;
}

}  // namespace detail

// --- Public entry point -----------------------------------------------------
// Convex-convex contact via GJK on the cores, with the rounding radii folded in.
//
// The Minkowski difference of two rounded shapes is the Minkowski difference of
// their cores, itself rounded by (rA + rB). So:
//   - GJK gives the core-to-core distance `d` and the core witness direction.
//   - If d > rA + rB the rounded shapes are separated -> no contact.
//   - If 0 < d <= rA + rB the cores are apart but the rounding shells overlap:
//     the normal is the GJK direction and penetration = (rA + rB) - d. No EPA
//     needed (this is the fast path that subsumes sphere/capsule contacts).
//   - If the cores themselves overlap, EPA recovers the core penetration; the
//     rounding radii then add to the depth.
//
// `normal` is unit A -> B (pushing B by +normal separates the pair). `point` is
// placed on the midline of the overlap region between the two surfaces.
template <class ShapeA, class ShapeB>
Contact gjkEpaContact(const ShapeA& a, const ShapeB& b) {
    const float rA = a.radius();
    const float rB = b.radius();
    const float rSum = rA + rB;

    Contact contact;
    const GjkResult gjk = detail::gjkDistance(a, b);

    if (!gjk.intersecting && gjk.distance > 1e-9f) {
        // Cores are separated. Rounded shells may still overlap.
        if (gjk.distance >= rSum) {
            return contact;  // not touching
        }
        contact.normal = gjk.normal;                 // A -> B
        contact.penetration = rSum - gjk.distance;
        // Surface points: move the core witnesses outward by each radius along n.
        const Vec3 surfA = gjk.pointA + contact.normal * rA;
        const Vec3 surfB = gjk.pointB - contact.normal * rB;
        contact.point = (surfA + surfB) * 0.5f;
        contact.touching = true;
        return contact;
    }

    // Cores overlap (or just touch): EPA recovers depth + normal of the cores.
    const detail::EpaResult epa = detail::epaPenetration(a, b, gjk);
    if (!epa.valid) {
        return contact;
    }
    contact.normal = epa.normal;                     // A -> B
    contact.penetration = epa.depth + rSum;
    const Vec3 surfA = epa.pointA + contact.normal * rA;
    const Vec3 surfB = epa.pointB - contact.normal * rB;
    contact.point = (surfA + surfB) * 0.5f;
    contact.touching = true;
    return contact;
}

}  // namespace xpbd::narrowphase

#endif  // XPBD_NARROWPHASE_GJK_HPP
