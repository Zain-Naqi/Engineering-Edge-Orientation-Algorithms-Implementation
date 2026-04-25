#include "Graph.h"
#include "Venkateswaran.h"
#include "FastImprove.h"
#include <iostream>
#include <cassert>
#include <chrono>

using namespace std;

using Clock = chrono::high_resolution_clock;
using Ms = chrono::duration<double, milli>;

// ─────────────────────────────────────────────────────────────────────────────
//  This file shows exactly how every algorithm comparison works:
//
//   1. Load graph ONCE.
//   2. Call g.fresh_orientation() to get the canonical starting state.
//   3. Pass a COPY of that state to each algorithm.
//   4. Each algorithm gets identical input → fair comparison.
//
//  When adding FastImprove, RPO, etc., we will follow this exact pattern.
// ─────────────────────────────────────────────────────────────────────────────


void print_orientation(const Graph& g, const OrientationState& s, int max_show = 20) {
    cout << " Edge orientations (first " << max_show << "):\n";
    for (int i = 0; i < min(g.m(), max_show); ++i) {
        int src = s.source(i, g.edges());
        int dst = s.dest(i, g.edges());
        cout << "   edge " << i << ": " << src << " ->  " << dst << " ( outdeg[" << src << "] = " << s.outdeg[src] << ")\n";
    }
}


// ── Helper: Test with both algorithms ─────────────────────────────────────────
void test_with_both_algorithms(const std::string& test_name, const Graph& g) {
    std::cout << "=== " << test_name << " ===\n";
    std::cout << "  n=" << g.n() << "  m=" << g.m() << "\n";

    OrientationState canonical = g.fresh_orientation();
    std::cout << "  Initial max_outdeg=" << canonical.max_outdeg() << "\n\n";

    // Scenario 1: Venkateswaran alone
    {
        OrientationState s = canonical;
        auto t0 = Clock::now();
        auto res = venkateswaran(g, s);
        double ms = Ms(Clock::now() - t0).count();

        std::cout << "  [Scenario 1] Venkateswaran alone:\n";
        std::cout << "    d*=" << res.d_star
                  << "  time=" << ms << "ms"
                  << "  paths_flipped=" << res.paths_flipped
                  << "  edges_visited=" << res.edges_visited << "\n";
    }

    // Scenario 2: FastImprove + Venkateswaran
    {
        OrientationState s = canonical;
        auto t0 = Clock::now();
        auto fi = fast_improve(g, s);
        auto res = venkateswaran(g, s);
        double ms = Ms(Clock::now() - t0).count();

        std::cout << "  [Scenario 2] FastImprove + Venkateswaran:\n";
        std::cout << "    d*=" << res.d_star
                  << "  time=" << ms << "ms"
                  << "  paths_flipped=" << res.paths_flipped
                  << "  edges_visited=" << res.edges_visited
                  << "  (FastImprove: flipped=" << fi.edges_flipped
                  << "  k_before=" << fi.initial_max_outdeg
                  << "  k_after=" << fi.final_max_outdeg << ")\n";
    }

    std::cout << "\n";
}

// ── Unit tests on tiny graphs ─────────────────────────────────────────────────
 
void test_triangle() {
    Graph g = Graph::from_pairs(3, {{0,1},{1,2},{0,2}});
    test_with_both_algorithms("Triangle Test", g);
}

void test_star() {
    Graph g = Graph::from_pairs(5, {{0,1},{0,2},{0,3},{0,4}});
    test_with_both_algorithms("Star K_(1,4) Test", g);
}

void test_path() {
    Graph g = Graph::from_pairs(5, {{0,1},{1,2},{2,3},{3,4}});
    test_with_both_algorithms("Path P_5 Test", g);
}

void test_complete_graph_k4() {
    Graph g = Graph::from_pairs(4, {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}});
    test_with_both_algorithms("Complete Graph K4 Test", g);
}

// ── Fair comparison demo ──────────────────────────────────────────────────────
//
//  This shows the exact pattern we will use when comparing algorithms.
//  Every algorithm gets the same OrientationState copy.

void demo_fair_comparison(const Graph& g, const std::string& name) {
    std::cout << "=== Fair comparison demo: " << name << " ===\n";
    std::cout << "  n=" << g.n() << "  m=" << g.m() << "\n";
 
    // The canonical starting orientation — produced once, copied per algorithm
    OrientationState canonical = g.fresh_orientation();
 
    // ── Algorithm 1: Venkateswaran ────────────────────────────────────────
    // {
    //     OrientationState s = canonical;   // <-- copy, not reference
    //     auto t0 = Clock::now();
    //     auto res = venkateswaran(g, s);
    //     double ms = Ms(Clock::now() - t0).count();
 
    //     std::cout << "  Venkateswaran:  d*=" << res.d_star
    //               << "  time=" << ms << "ms"
    //               << "  paths=" << res.paths_flipped
    //               << "  edge_visits=" << res.edges_visited << "\n";
    // }
 
    // When we implement FastImprove+DFS and RPO, add them here:
    // {
    //     OrientationState s = canonical;   // same starting state
    //     auto t0 = Clock::now();
    //     auto res = rpo(g, s);
    //     ...
    // }
    
    //Engineering Approach 1: FastImprove + Venkateswaran
    {
        OrientationState s = canonical;   // <-- copy, not reference
        auto t0 = Clock::now();
        auto fi = fast_improve(g, s);
        auto res = venkateswaran(g, s);

        double ms = Ms(Clock::now() - t0).count();

        std::cout << "  FastImprove+Venkateswaran:  d*=" << res.d_star
                << "  time=" << ms << "ms"
                << "  paths=" << res.paths_flipped
                << "  edge_visits=" << res.edges_visited
                << "  fi_flipped=" << fi.edges_flipped
                << "  start_k=" << fi.initial_max_outdeg
                << "  after_fi_k=" << fi.final_max_outdeg << "\n";

    }
 
    std::cout << "\n";
}


// Main

int main(int argc, char* argv[]) {
    // Unit tests
    test_triangle();
    test_star();
    test_path();    
    test_complete_graph_k4();

    if (argc >= 2) {
        try {
            Graph g = Graph::from_edge_list(argv[1]);
            string name = argv[1];
            demo_fair_comparison(g, name);
        } catch (const exception& e) {
            cerr << "Error: " << e.what() << "\n";
            return 1;
        }
    } else {
        cout << "Usage ./run <graph_file.txt>\n";
        cout << "(Unit tests passed.  Provide a graph file to benchmark.)\n";
    }

    return 0;
}



