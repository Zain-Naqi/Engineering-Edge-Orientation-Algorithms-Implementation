CXX      = g++
CXXFLAGS = -std=c++20 -O2 -Wall -Wextra

# Use -O3 and -DNDEBUG for benchmarks (disables assert)
BENCH_FLAGS = -std=c++20 -O3 -DNDEBUG -Wall

SRC = Graph.cpp Run.cpp
OUT = run

all:
	$(CXX) $(CXXFLAGS) $(SRC) -o $(OUT)

bench:
	$(CXX) $(BENCH_FLAGS) $(SRC) -o $(OUT)_bench

clean:
	rm -f $(OUT) $(OUT)_bench