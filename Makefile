CXX       = g++
CXXFLAGS  = -std=c++20 -O2 -Wall -Wextra
BENCHFLAGS= -std=c++20 -O3 -DNDEBUG -Wall

# Source files shared by all targets
SHARED_SRC = Graph.cpp

# ── Primary target: RPO test runner (debug build) ─────────────────────────────
run: $(SHARED_SRC) Run_RPO.cpp
	$(CXX) $(CXXFLAGS) $(SHARED_SRC) Run_RPO.cpp -o run

# ── Benchmark build (asserts off, full optimisation) ─────────────────────────
bench: $(SHARED_SRC) Run_RPO.cpp
	$(CXX) $(BENCHFLAGS) $(SHARED_SRC) Run_RPO.cpp -o run_bench

# ── Legacy runners (kept for reference) ──────────────────────────────────────
run_venkat: $(SHARED_SRC) Run_Venkateswaran.cpp
	$(CXX) $(CXXFLAGS) $(SHARED_SRC) Run_Venkateswaran.cpp -o run_venkat

run_dfs: $(SHARED_SRC) Run_OptimizedDFS.cpp
	$(CXX) $(CXXFLAGS) $(SHARED_SRC) Run_OptimizedDFS.cpp -o run_dfs

# ── Clean ─────────────────────────────────────────────────────────────────────
clean:
	rm -f run run_bench run_venkat run_dfs

.PHONY: all run bench run_venkat run_dfs clean
all: run
