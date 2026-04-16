#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

// ─────────────────────────────────────────────────────────────────────────────
// nfp_core.h  —  No-Fit Polygon engine
//
// WHAT THIS FILE IS:
//   The entire C++ core. One header, no dependencies beyond the STL.
//   Takes polygons as plain vertex lists. Returns NFP polygons.
//   Nothing else.
//
// ASSUMPTIONS:
//   - All polygons are closed, CCW-wound, no repeated closing vertex.
//   - Units: millimetres, double precision.
//   - Input shapes are already clean polygons (no curves, no arcs).
//     Curve-to-polygon conversion happens in Python before this is called.
//   - Reference point of shape B = B[0] (first vertex).
//     The NFP tells you where B[0] can be placed so A and B touch
//     but do not overlap.
//   - Shapes with holes are not supported in V1. Add later if needed.
//
// PUBLIC API (three functions):
//   computeNFP(A, B)        → NFP polygon
//   computeBinIFP(bin, B)   → valid region for B[0] inside the bin
//   offset(poly, delta)     → expand polygon outward by delta mm (clearance)
// ─────────────────────────────────────────────────────────────────────────────

namespace nfp {

// ── Types ────────────────────────────────────────────────────────────────────

struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
    Point operator+(const Point& o) const { return {x+o.x, y+o.y}; }
    Point operator-(const Point& o) const { return {x-o.x, y-o.y}; }
    Point operator*(double s)        const { return {x*s,   y*s};   }
};

using Poly = std::vector<Point>;

// ── Constants ─────────────────────────────────────────────────────────────────

constexpr double EPS    = 1e-9;
constexpr double TWO_PI = 6.283185307179586;

// ── Basic geometry ────────────────────────────────────────────────────────────

inline double cross(Point a, Point b)         { return a.x*b.y - a.y*b.x; }
inline double dot(Point a, Point b)           { return a.x*b.x + a.y*b.y; }
inline double norm(Point a)                   { return std::sqrt(a.x*a.x + a.y*a.y); }
inline double dist(Point a, Point b)          { return norm(b-a); }
inline double angle(Point p)                  { return std::atan2(p.y, p.x); }

inline double signedArea(const Poly& p) {
    double a = 0;
    for (int i = 0, n = p.size(); i < n; ++i)
        a += cross(p[i], p[(i+1)%n]);
    return a * 0.5;
}

inline bool isCCW(const Poly& p) { return signedArea(p) > 0; }

inline void ensureCCW(Poly& p) {
    if (!isCCW(p)) std::reverse(p.begin(), p.end());
}

inline bool pointInPoly(Point pt, const Poly& p) {
    int c = 0, n = p.size();
    for (int i = 0, j = n-1; i < n; j = i++) {
        if (((p[i].y > pt.y) != (p[j].y > pt.y)) &&
            pt.x < (p[j].x-p[i].x)*(pt.y-p[i].y)/(p[j].y-p[i].y)+p[i].x)
            c++;
    }
    return c & 1;
}

// ── Convex hull (Graham scan) ─────────────────────────────────────────────────

inline Poly convexHull(Poly pts) {
    int n = pts.size();
    if (n < 3) return pts;

    // Find bottom-left pivot
    int pivot = 0;
    for (int i = 1; i < n; ++i)
        if (pts[i].y < pts[pivot].y ||
           (pts[i].y == pts[pivot].y && pts[i].x < pts[pivot].x))
            pivot = i;
    std::swap(pts[0], pts[pivot]);
    Point p0 = pts[0];

    std::sort(pts.begin()+1, pts.end(), [&](const Point& a, const Point& b) {
        double c = cross(a-p0, b-p0);
        if (std::abs(c) > EPS) return c > 0;
        return dist(p0,a) < dist(p0,b);
    });

    Poly hull;
    for (auto& p : pts) {
        while (hull.size() >= 2 &&
               cross(hull.back()-hull[hull.size()-2], p-hull[hull.size()-2]) <= EPS)
            hull.pop_back();
        hull.push_back(p);
    }
    return hull;
}

// ── Convexity check ───────────────────────────────────────────────────────────

inline bool isConvex(const Poly& p) {
    int n = p.size();
    for (int i = 0; i < n; ++i)
        if (cross(p[(i+1)%n]-p[i], p[(i+2)%n]-p[(i+1)%n]) < -EPS)
            return false;
    return true;
}

// ── Convex decomposition ──────────────────────────────────────────────────────
// Splits a non-convex polygon into convex parts.
// Strategy: find a reflex vertex, split across the best diagonal, recurse.
// Conservative — returns hull approximation if no valid diagonal found.

inline bool validDiag(const Poly& poly, int i, int j) {
    int n = poly.size();
    if (std::abs(i-j) <= 1 || std::abs(i-j) >= n-1) return false;
    Point pi = poly[i], pj = poly[j];
    Point mid = {(pi.x+pj.x)*0.5, (pi.y+pj.y)*0.5};
    if (!pointInPoly(mid, poly)) return false;
    for (int k = 0; k < n; ++k) {
        int l = (k+1)%n;
        if (k==i||k==j||l==i||l==j) continue;
        double d1 = cross(pj-pi, poly[k]-pi), d2 = cross(pj-pi, poly[l]-pi);
        double d3 = cross(poly[l]-poly[k], pi-poly[k]);
        double d4 = cross(poly[l]-poly[k], pj-poly[k]);
        if (((d1>EPS&&d2<-EPS)||(d1<-EPS&&d2>EPS)) &&
            ((d3>EPS&&d4<-EPS)||(d3<-EPS&&d4>EPS))) return false;
    }
    return true;
}

inline std::vector<Poly> decompose(const Poly& poly);

inline std::vector<Poly> splitAndDecompose(const Poly& poly, int i, int j) {
    if (i > j) std::swap(i, j);
    int n = poly.size();
    Poly p1, p2;
    for (int k = i; k <= j; ++k) p1.push_back(poly[k]);
    for (int k = j; k <= n+i; ++k) p2.push_back(poly[k%n]);
    auto d1 = decompose(p1), d2 = decompose(p2);
    d1.insert(d1.end(), d2.begin(), d2.end());
    return d1;
}

inline std::vector<Poly> decompose(const Poly& poly) {
    if (isConvex(poly)) return {poly};
    int n = poly.size();
    for (int i = 0; i < n; ++i) {
        // Reflex vertex at i+1
        Point a = poly[i], b = poly[(i+1)%n], c = poly[(i+2)%n];
        if (cross(b-a, c-b) >= -EPS) continue;
        int vi = (i+1)%n;
        for (int j = 0; j < n; ++j)
            if (validDiag(poly, vi, j))
                return splitAndDecompose(poly, vi, j);
    }
    // Fallback: convex hull (slightly over-approximates forbidden zone)
    Poly hull = convexHull(poly);
    ensureCCW(hull);
    return {hull};
}

// ── Minkowski sum of two CONVEX CCW polygons ──────────────────────────────────
// NFP(A, B) = A ⊕ (−B)
// −B = B reflected through B[0] (its reference point)
// Algorithm: merge edge vectors of A and −B sorted by angle, walk them.

inline Poly minkowskiConvex(const Poly& A, const Poly& B_neg) {
    // Build (edgeVector, angle) for each polygon
    auto edges = [](const Poly& p) {
        std::vector<std::pair<Point,double>> ev;
        int n = p.size();
        for (int i = 0; i < n; ++i) {
            Point v = p[(i+1)%n] - p[i];
            if (norm(v) < EPS) continue;
            ev.push_back({v, angle(v)});
        }
        return ev;
    };

    auto eA = edges(A), eB = edges(B_neg);
    if (eA.empty() || eB.empty()) return {};

    // Normalise angles to [0, 2π)
    for (auto& e : eA) { if (e.second < 0) e.second += TWO_PI; }
    for (auto& e : eB) { if (e.second < 0) e.second += TWO_PI; }

    // Starting point: bottom-most vertex of A + bottom-most of B_neg
    auto botmost = [](const Poly& p) {
        int idx = 0;
        for (int i = 1; i < (int)p.size(); ++i)
            if (p[i].y < p[idx].y || (p[i].y == p[idx].y && p[i].x < p[idx].x))
                idx = i;
        return idx;
    };

    // Rotate edge lists to start from bottom-most vertex
    auto rot = [](std::vector<std::pair<Point,double>>& ev, int start) {
        std::rotate(ev.begin(), ev.begin()+start, ev.end());
    };
    rot(eA, botmost(A));
    rot(eB, botmost(B_neg));

    Point cur = A[botmost(A)] + B_neg[botmost(B_neg)];
    Poly nfp;
    nfp.push_back(cur);

    int ia = 0, ib = 0, na = eA.size(), nb = eB.size();
    while (ia < na || ib < nb) {
        double angA = ia < na ? eA[ia].second : 1e18;
        double angB = ib < nb ? eB[ib].second : 1e18;
        Point step = {0, 0};
        if (angA <= angB + EPS && ia < na) {
            step = step + eA[ia++].first;
            if (ib < nb && std::abs(angA - angB) < EPS)
                step = step + eB[ib++].first;
        } else {
            step = step + eB[ib++].first;
        }
        cur = cur + step;
        if (dist(nfp.back(), cur) > EPS) nfp.push_back(cur);
    }

    // Remove closing duplicate
    if (nfp.size() >= 2 && dist(nfp.front(), nfp.back()) < EPS)
        nfp.pop_back();

    ensureCCW(nfp);
    return nfp;
}

// ── Reflect B through B[0] ────────────────────────────────────────────────────
// Produces −B: each vertex maps to B[0] - (v - B[0])

inline Poly reflectB(const Poly& B) {
    Poly neg;
    neg.reserve(B.size());
    for (auto& v : B)
        neg.push_back({2*B[0].x - v.x, 2*B[0].y - v.y});
    ensureCCW(neg);
    return neg;
}

// ── Union of polygons (convex hull approximation) ─────────────────────────────
// Correct for convex NFP parts. For non-convex unions, replace with Clipper2.
// The hull over-approximates the forbidden zone — conservative, never allows
// a real overlap through.

inline Poly hullUnion(const std::vector<Poly>& polys) {
    if (polys.empty()) return {};
    if (polys.size() == 1) return polys[0];
    Poly all;
    for (auto& p : polys) all.insert(all.end(), p.begin(), p.end());
    Poly h = convexHull(all);
    ensureCCW(h);
    return h;
}

// ── PUBLIC API ────────────────────────────────────────────────────────────────

// computeNFP: returns the No-Fit Polygon for A (fixed) and B (orbiting).
// B[0] placed on any point of the returned polygon = touching, non-overlapping.
// B[0] inside the returned polygon = overlapping.

inline Poly computeNFP(Poly A, Poly B) {
    ensureCCW(A);
    ensureCCW(B);

    auto partsA = decompose(A);
    auto partsB = decompose(B);
    Poly B_neg  = reflectB(B);
    auto partsB_neg = decompose(B_neg);

    std::vector<Poly> parts;
    for (auto& pa : partsA) {
        ensureCCW(pa);
        for (auto& pb_neg : partsB_neg) {
            ensureCCW(pb_neg);
            if (std::abs(signedArea(pa)) < EPS) continue;
            if (std::abs(signedArea(pb_neg)) < EPS) continue;
            auto nfp = minkowskiConvex(pa, pb_neg);
            if (nfp.size() >= 3) parts.push_back(nfp);
        }
    }

    if (parts.empty()) return {};
    auto result = hullUnion(parts);
    ensureCCW(result);
    return result;
}

// computeBinIFP: returns the region where B[0] must lie for B to stay
// entirely inside the bin rectangle.
// Simple bounding-box erosion — exact for rectangular bins.

inline Poly computeBinIFP(const Poly& bin, Poly B) {
    ensureCCW(B);

    // Find B's extent relative to B[0]
    double minDx=0, maxDx=0, minDy=0, maxDy=0;
    for (auto& v : B) {
        minDx = std::min(minDx, v.x - B[0].x);
        maxDx = std::max(maxDx, v.x - B[0].x);
        minDy = std::min(minDy, v.y - B[0].y);
        maxDy = std::max(maxDy, v.y - B[0].y);
    }

    // Bin bounds
    double bx0=bin[0].x, by0=bin[0].y, bx1=bx0, by1=by0;
    for (auto& v : bin) {
        bx0 = std::min(bx0, v.x); by0 = std::min(by0, v.y);
        bx1 = std::max(bx1, v.x); by1 = std::max(by1, v.y);
    }

    double x0 = bx0 - minDx;
    double y0 = by0 - minDy;
    double x1 = bx1 - maxDx;
    double y1 = by1 - maxDy;

    if (x0 >= x1 || y0 >= y1) return {};  // B doesn't fit

    Poly ifp = {{x0,y0},{x1,y0},{x1,y1},{x0,y1}};
    ensureCCW(ifp);
    return ifp;
}

// offset: expand a polygon outward by delta (Minkowski sum with a circle
// approximated as a regular polygon — 16 sides is sufficient for clearance).
// For production quality, replace with pyclipper on the Python side.
// This C++ version is provided as a fallback only.

inline Poly offset(const Poly& poly, double delta, int circle_pts = 16) {
    if (delta < EPS) return poly;
    std::vector<Poly> parts;
    // Offset each edge outward
    int n = poly.size();
    for (int i = 0; i < n; ++i) {
        Point a = poly[i], b = poly[(i+1)%n];
        Point edge = b - a;
        double len = norm(edge);
        if (len < EPS) continue;
        Point normal = {-edge.y/len * delta, edge.x/len * delta};
        parts.push_back({a + normal, b + normal, b, a});
    }
    // Add circle approximations at each vertex
    for (auto& v : poly) {
        Poly circ;
        for (int j = 0; j < circle_pts; ++j) {
            double a = TWO_PI * j / circle_pts;
            circ.push_back({v.x + delta*std::cos(a), v.y + delta*std::sin(a)});
        }
        parts.push_back(circ);
    }
    auto result = hullUnion(parts);
    ensureCCW(result);
    return result;
}

} // namespace nfp
