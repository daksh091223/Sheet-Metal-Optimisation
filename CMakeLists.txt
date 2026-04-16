cmake_minimum_required(VERSION 3.15)
project(nfp_engine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# ── pybind11 ─────────────────────────────────────
execute_process(
    COMMAND python -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")

# ✅ FIXED Python detection
set(PYBIND11_FINDPYTHON ON)
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# ── Python module ────────────────────────────────
pybind11_add_module(nfp_engine bindings.cpp)
target_include_directories(nfp_engine PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# ── C++ test ─────────────────────────────────────
add_executable(test_nfp_core test_nfp_core.cpp)
target_include_directories(test_nfp_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})