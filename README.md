# Sheet Metal Nesting & Optimization Engine
A high-performance **sheet metal nesting engine** built using a hybrid **C++ (geometry core) + Python (optimization layer)** architecture.
The system minimizes material wastage by computing optimal layouts using **No-Fit Polygons (NFP)** and heuristic + evolutionary strategies.

---

## Highlights
* C++ Geometry Core for fast NFP / IFP computation
* Hybrid Optimization
  * Greedy bottom-left placement
  * Genetic Algorithm (GA) for layout improvement
* Manufacturing-aware constraints
  * Kerf / clearance handling via polygon offset
  * Rotation control (grain constraints supported)
* SVG-based visualization of final layouts
* Python fallback engine (no compilation required)
---

## Project Structure
├── bindings.cpp        # Python ↔ C++ bridge (pybind11)
├── nfp_core.hpp        # Core geometry engine
├── placer.py           # Nesting + optimization logic
├── demo.py             # CLI + visualization
├── demo_2.py           # Rectangular test cases
├── experimental.py     # Experimental runs
├── test_nfp_core.cpp   # C++ validation tests
├── CMakeLists.txt      # Build configuration
├── *.svg               # Output layouts
---

## Setup
### Install dependencies
pip install pybind11

### Build (recommended)
mkdir build
cd build
cmake ..
cmake --build . --config Release

If skipped, the engine automatically falls back to a Python implementation (slower).
---

## Usage
### Basic run
python demo.py

### With Genetic Algorithm
python demo.py --ga

### Advanced controls
python demo.py --rotations 8 --clearance 1.5 --margin 5

### Custom input
python demo.py shapes.json

---

## Core Idea
The engine transforms nesting into a geometric feasibility + optimization problem:
* NFP (No-Fit Polygon) → defines collision boundaries
* IFP (Inner Fit Polygon) → ensures shapes stay inside the sheet
* Offset geometry → enforces cutting clearance
* Optimization strategies:
  * Greedy → fast baseline
  * GA → improved packing density
  * 
This ensures non-overlapping, manufacturable layouts with high material utilization.

---

## Output
Generated SVG files include:
* Final placement layout
* Rotation annotations
* Utilization percentage
* GA convergence (if enabled)

Examples:
nesting_result.svg
rect_greedy.svg
rect_ga.svg

---

## Validation
C++ test suite verifies:
* NFP correctness
* Convexity & hull logic
* Offset behavior
* Bin fitting constraints

Run:
g++ -std=c++17 -O2 test_nfp_core.cpp -o test
./test

---

## Applications
* Sheet metal cutting (laser / plasma / waterjet)
* CNC nesting optimization
* Industrial fabrication workflows
* Material cost reduction systems

---

## Future Scope
* DXF / CAD file support
* Multi-sheet optimization
* Parallelized GA
* Interactive UI / web tool

---

## Project Context
Developed as part of a Mechanical Engineering course project, integrating manufacturing principles with computational optimization to solve real-world sheet utilization problems.

---
