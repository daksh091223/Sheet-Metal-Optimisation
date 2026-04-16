// test_nfp_core.cpp  —  standalone C++ verification, no Python needed
// Build: g++ -std=c++17 -O2 -o test_nfp_core test_nfp_core.cpp
// Run:   ./test_nfp_core

#include "nfp_core.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace nfp;

static bool near(double a, double b, double eps=1e-4) {
    return std::abs(a-b) < eps;
}

static Poly square(double x, double y, double s) {
    Poly p = {{x,y},{x+s,y},{x+s,y+s},{x,y+s}};
    ensureCCW(p);
    return p;
}

static Poly rect(double x, double y, double w, double h) {
    Poly p = {{x,y},{x+w,y},{x+w,y+h},{x,y+h}};
    ensureCCW(p);
    return p;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

void test_signed_area() {
    Poly sq = square(0,0,4);
    assert(near(signedArea(sq), 16.0));
    assert(isCCW(sq));
    std::cout << "  [ok] signed area\n";
}

void test_convex_hull() {
    // Square + interior point — hull should still be 4 vertices
    Poly pts = {{0,0},{4,0},{4,4},{0,4},{2,2}};
    Poly h = convexHull(pts);
    assert(h.size() == 4);
    std::cout << "  [ok] convex hull\n";
}

void test_is_convex() {
    Poly sq = square(0,0,4);
    assert(isConvex(sq));
    // L-shape — non-convex
    Poly L = {{0,0},{4,0},{4,2},{2,2},{2,4},{0,4}};
    ensureCCW(L);
    assert(!isConvex(L));
    std::cout << "  [ok] convex check\n";
}

void test_reflect_b() {
    // Unit square at origin: B[0]=(0,0), B[1]=(1,0)
    // Reflected: B[0]=(0,0) stays, B[1] → (0,0)-(1,0)-(0,0) = (-1,0)
    Poly B = {{0,0},{1,0},{1,1},{0,1}};
    Poly neg = reflectB(B);
    assert(neg.size() == 4);
    // Check one reflected point exists
    bool found = false;
    for (auto& v : neg)
        if (near(v.x,-1.0) && near(v.y,0.0)) { found=true; break; }
    assert(found);
    std::cout << "  [ok] reflect B\n";
}

void test_nfp_two_squares() {
    // NFP of two s×s squares = 2s×2s square, area = 4s²
    Poly A = square(0,0,4);
    Poly B = square(0,0,4);
    Poly nfp = computeNFP(A, B);
    assert(nfp.size() >= 4);
    double a = std::abs(signedArea(nfp));
    // Area of NFP for two 4×4 squares = 8×8 = 64
    assert(near(a, 64.0, 1.0));
    assert(isCCW(nfp));
    std::cout << "  [ok] NFP two 4×4 squares (area=" << a << ")\n";
}

void test_nfp_rect_square() {
    // 6×4 rectangle A, 2×2 square B
    Poly A = rect(0,0,6,4);
    Poly B = square(0,0,2);
    Poly nfp = computeNFP(A, B);
    assert(nfp.size() >= 4);
    assert(isCCW(nfp));
    // Area should be (6+2)×(4+2) = 48
    double a = std::abs(signedArea(nfp));
    assert(near(a, 48.0, 1.0));
    std::cout << "  [ok] NFP rect×square (area=" << a << ")\n";
}

void test_nfp_inside_means_overlap() {
    Poly A = square(0,0,10);
    Poly B = square(0,0,4);
    Poly nfp = computeNFP(A, B);
    // Point well inside NFP = B overlaps A
    assert(pointInPoly({5,5}, nfp));
    // Point well outside NFP = B clear of A
    assert(!pointInPoly({50,50}, nfp));
    std::cout << "  [ok] NFP overlap semantics\n";
}

void test_nfp_l_shape() {
    // Non-convex L-shape A, small square B
    Poly L = {{0,0},{6,0},{6,3},{3,3},{3,6},{0,6}};
    Poly B = square(0,0,2);
    ensureCCW(L); ensureCCW(B);
    Poly nfp = computeNFP(L, B);
    assert(nfp.size() >= 3);
    assert(std::abs(signedArea(nfp)) > 0);
    std::cout << "  [ok] NFP L-shape×square (verts="
              << nfp.size() << ")\n";
}

void test_bin_ifp_fits() {
    // 200×100 bin, 40×40 part
    // IFP should be (200-40)×(100-40) = 160×60
    Poly bin = rect(0,0,200,100);
    Poly B   = square(0,0,40);
    Poly ifp = computeBinIFP(bin, B);
    assert(ifp.size() == 4);
    double w=0, h=0;
    double x0=ifp[0].x, x1=ifp[0].x, y0=ifp[0].y, y1=ifp[0].y;
    for (auto& v:ifp){x0=std::min(x0,v.x);x1=std::max(x1,v.x);y0=std::min(y0,v.y);y1=std::max(y1,v.y);}
    w=x1-x0; h=y1-y0;
    assert(near(w,160.0) && near(h,60.0));
    std::cout << "  [ok] bin IFP (" << w << "×" << h << ")\n";
}

void test_bin_ifp_no_fit() {
    // Part larger than bin — should return empty
    Poly bin = rect(0,0,10,10);
    Poly B   = square(0,0,20);
    Poly ifp = computeBinIFP(bin, B);
    assert(ifp.empty());
    std::cout << "  [ok] bin IFP empty when part too large\n";
}

void test_offset() {
    Poly sq = square(0,0,10);
    Poly off = offset(sq, 2.0);
    assert(off.size() >= 4);
    // Offset polygon must be larger
    assert(std::abs(signedArea(off)) > std::abs(signedArea(sq)));
    std::cout << "  [ok] offset expands polygon\n";
}

void test_decompose_convex_unchanged() {
    Poly sq = square(0,0,4);
    auto parts = decompose(sq);
    assert(parts.size() == 1);
    std::cout << "  [ok] decompose: convex unchanged\n";
}

void test_decompose_l_shape() {
    Poly L = {{0,0},{6,0},{6,3},{3,3},{3,6},{0,6}};
    ensureCCW(L);
    auto parts = decompose(L);
    assert(parts.size() >= 2);
    // Total area conserved
    double total = 0;
    for (auto& p : parts) total += std::abs(signedArea(p));
    assert(near(total, std::abs(signedArea(L)), 0.1));
    std::cout << "  [ok] decompose: L-shape → " << parts.size()
              << " parts, area conserved\n";
}

int main() {
    std::cout << "=== nfp_core tests ===\n";
    test_signed_area();
    test_convex_hull();
    test_is_convex();
    test_reflect_b();
    test_nfp_two_squares();
    test_nfp_rect_square();
    test_nfp_inside_means_overlap();
    test_nfp_l_shape();
    test_bin_ifp_fits();
    test_bin_ifp_no_fit();
    test_offset();
    test_decompose_convex_unchanged();
    test_decompose_l_shape();
    std::cout << "=== all passed ===\n";
    return 0;
}
