// bindings.cpp  —  pybind11 bridge between nfp_core and Python
//
// Python interface:
//   import nfp_engine as nfp
//
//   # Polygons are lists of (x, y) tuples
//   A = [(0,0), (100,0), (100,80), (0,80)]      # 100x80 rectangle
//   B = [(0,0), (40,0),  (40,40),  (0,40)]      # 40x40 square
//
//   nfp  = nfp.compute_nfp(A, B)                # → list of (x,y)
//   ifp  = nfp.compute_bin_ifp(bin_rect, B)     # → list of (x,y)
//   offB = nfp.offset(B, 2.5)                   # → list of (x,y)

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "nfp_core.hpp"

namespace py = pybind11;

// Convert Python list-of-tuples ↔ nfp::Poly
using PyPoly = std::vector<std::pair<double,double>>;

static nfp::Poly fromPy(const PyPoly& pts) {
    nfp::Poly p;
    p.reserve(pts.size());
    for (auto& [x,y] : pts) p.push_back({x,y});
    return p;
}

static PyPoly toPy(const nfp::Poly& p) {
    PyPoly out;
    out.reserve(p.size());
    for (auto& v : p) out.push_back({v.x, v.y});
    return out;
}

PYBIND11_MODULE(nfp_engine, m) {
    m.doc() = "NFP nesting engine — C++ core exposed to Python";

    m.def("compute_nfp",
        [](const PyPoly& A, const PyPoly& B) {
            return toPy(nfp::computeNFP(fromPy(A), fromPy(B)));
        },
        py::arg("A"), py::arg("B"),
        R"doc(
Compute the No-Fit Polygon for shape A (fixed) and shape B (orbiting).

Args:
    A: list of (x, y) tuples — fixed shape, CCW wound
    B: list of (x, y) tuples — orbiting shape, CCW wound

Returns:
    list of (x, y) tuples — NFP boundary, CCW wound
    B[0] placed ON this boundary → touching, non-overlapping
    B[0] placed INSIDE this boundary → overlapping
        )doc");

    m.def("compute_bin_ifp",
        [](const PyPoly& bin, const PyPoly& B) {
            return toPy(nfp::computeBinIFP(fromPy(bin), fromPy(B)));
        },
        py::arg("bin"), py::arg("B"),
        R"doc(
Compute the Inner Fit Polygon: region where B[0] must lie for B to stay
fully inside the bin.

Args:
    bin: list of (x, y) tuples — bin boundary (rectangle)
    B:   list of (x, y) tuples — shape to place

Returns:
    list of (x, y) tuples — valid region for B[0], or [] if B doesn't fit
        )doc");

    m.def("offset",
        [](const PyPoly& poly, double delta) {
            return toPy(nfp::offset(fromPy(poly), delta));
        },
        py::arg("poly"), py::arg("delta"),
        R"doc(
Expand a polygon outward by delta mm (clearance / kerf offset).
For production quality use pyclipper.offset() on the Python side instead.

Args:
    poly:  list of (x, y) tuples
    delta: offset distance in mm (same units as polygon coords)

Returns:
    list of (x, y) tuples — expanded polygon
        )doc");
}
