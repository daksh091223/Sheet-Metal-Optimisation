CXX = g++
CXXFLAGS = -O3 -Wall -shared -std=c++17 -fPIC

PYTHON = python
PYBIND11_INCLUDES = $(shell $(PYTHON) -m pybind11 --includes)
PYTHON_EXT_SUFFIX = .pyd   # <-- IMPORTANT CHANGE

MODULE_NAME = nfp_engine
MODULE_SRC = bindings.cpp

$(MODULE_NAME)$(PYTHON_EXT_SUFFIX): $(MODULE_SRC)
	$(CXX) $(CXXFLAGS) $(PYBIND11_INCLUDES) $^ -o $@

TEST_NAME = test_nfp_core
TEST_SRC = test_nfp_core.cpp

$(TEST_NAME): $(TEST_SRC)
	$(CXX) -O3 -std=c++17 $^ -o $@

clean:
	del *.pyd *.exe 2>nul