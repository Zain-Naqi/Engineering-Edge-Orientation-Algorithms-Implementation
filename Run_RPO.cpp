// ─────────────────────────────────────────────────────────────────────────────
//  Run_RPO.cpp  —  Test runner for the full RPO pipeline
//
//  Runs every hardcoded test graph through four algorithm configurations:
//
//    [1] Venkateswaran (BFS)  — baseline from the paper
//    [2] Venkateswaran (DFS)  — DFS variant
//    [3] FastImprove + OptimizedDFS
//    [4] RPO (full pipeline)  — 2-Approx* + FastImprove + EPS + cleanup
//                               (* only when density > 10)
//
//  Every algorithm receives an identical copy of the same initial orientation
//  so timings are directly comparable.
//
//  Compile (debug):
//    g++ -std=c++20 -O2 -Wall -Wextra  Graph.cpp Run_RPO.cpp -o run
//
//  Compile (benchmark — disables assert):
//    g++ -std=c++20 -O3 -DNDEBUG -Wall  Graph.cpp Run_RPO.cpp -o run_bench
//
//  Run unit tests only:
//    ./run
//
//  Run unit tests + external graph file:
//    ./run path/to/graph.txt
//
//  Graph file format: one edge per line  "u v"  (0-indexed, comments with #)
// ─────────────────────────────────────────────────────────────────────────────

#include "Graph.h"
#include "Venkateswaran.h"
#include "Venkateswaran_DFS.h"
#include "OptimizedDFS.h"
#include "FastImprove.h"
#include "RPO.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include <cassert>

using namespace std;
using Clock = chrono::high_resolution_clock;
using Ms    = chrono::duration<double, milli>;

// ─────────────────────────────────────────────────────────────────────────────
//  Formatting helpers
// ─────────────────────────────────────────────────────────────────────────────

static void print_separator(char c = '-', int width = 72) {
    cout << string(width, c) << "\n";
}

static void print_header(const string& title) {
    print_separator('=');
    cout << "  " << title << "\n";
    print_separator('=');
}

// ─────────────────────────────────────────────────────────────────────────────
//  Core comparison function
//  Runs all four algorithm configurations on graph g and prints results.
// ─────────────────────────────────────────────────────────────────────────────

void run_all(const string& test_name, const Graph& g) {
    print_header(test_name);
    cout << "  n=" << g.n()
         << "  m=" << g.m()
         << "  density=" << fixed << setprecision(2) << g.density()
         << "\n\n";

    // Canonical starting orientation — every edge {u,v} oriented u→v
    OrientationState canonical = g.fresh_orientation();
    cout << "  Initial max_outdeg = " << canonical.max_outdeg() << "\n\n";

    // ── [1] Venkateswaran BFS ─────────────────────────────────────────────
    {
        OrientationState s = canonical;
        auto t0  = Clock::now();
        auto res = venkateswaran(g, s);
        double ms = Ms(Clock::now() - t0).count();

        cout << "  [1] Venkateswaran (BFS)\n"
             << "      d*="              << res.d_star
             << "  time="                << fixed << setprecision(3) << ms << "ms"
             << "  paths_flipped="       << res.paths_flipped
             << "  edges_visited="       << res.edges_visited << "\n";
    }

    // ── [2] Venkateswaran DFS ─────────────────────────────────────────────
    {
        OrientationState s = canonical;
        auto t0  = Clock::now();
        auto res = venkateswaran_dfs(g, s);
        double ms = Ms(Clock::now() - t0).count();

        cout << "  [2] Venkateswaran (DFS)\n"
             << "      d*="              << res.d_star
             << "  time="                << fixed << setprecision(3) << ms << "ms"
             << "  paths_flipped="       << res.paths_flipped
             << "  edges_visited="       << res.edges_visited << "\n";
    }

    // ── [3] FastImprove + OptimizedDFS ────────────────────────────────────
    {
        OrientationState s = canonical;
        auto t0 = Clock::now();
        auto fi  = fast_improve(g, s);
        auto res = optimized_dfs(g, s);
        double ms = Ms(Clock::now() - t0).count();

        cout << "  [3] FastImprove + OptimizedDFS\n"
             << "      d*="              << res.d_star
             << "  time="                << fixed << setprecision(3) << ms << "ms"
             << "  paths_flipped="       << res.paths_flipped
             << "  edges_visited="       << res.edges_visited
             << "\n"
             << "      FastImprove: edges_flipped=" << fi.edges_flipped
             << "  k_before="  << fi.initial_max_outdeg
             << "  k_after="   << fi.final_max_outdeg << "\n";
    }

    // ── [4] RPO — full pipeline ───────────────────────────────────────────
    {
        OrientationState s = canonical;
        auto t0  = Clock::now();
        auto res = rpo(g, s);
        double ms = Ms(Clock::now() - t0).count();

        cout << "  [4] RPO (RapidPathOrientation — full pipeline)\n"
             << "      d*="    << res.d_star
             << "  time="      << fixed << setprecision(3) << ms << "ms"
             << "\n";

        if (res.d_after_two_approx >= 0) {
            cout << "      2-Approx:      d_approx=" << res.d_after_two_approx
                 << "  edges_flipped="  << res.two_approx_flipped << "\n";
        } else {
            cout << "      2-Approx:      skipped (density <= 10)\n";
        }

        cout << "      FastImprove:   edges_flipped=" << res.fast_improve_flipped
             << "  k_after=" << res.d_after_fast_improve << "\n"
             << "      EPS+cleanup:   paths_flipped=" << res.eps_paths_flipped
             << "  edges_visited=" << res.eps_edges_visited
             << "  k_after=" << res.d_after_eps << "\n";
    }

    cout << "\n";
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hardcoded test graphs
// ─────────────────────────────────────────────────────────────────────────────

// 3 vertices, 3 edges — optimal d* = 1
void test_triangle() {
    run_all("Triangle (K3)  |  expected d*=1",
            Graph::from_pairs(3, {{0,1},{1,2},{0,2}}));
}

// Star with 4 leaves — all edges incident to vertex 0
// Initial orientation: 0→1, 0→2, 0→3, 0→4  (d*=4)
// Optimal: each edge flipped alternately  → d*=1 (only one centre)
void test_star() {
    run_all("Star K(1,4)  |  expected d*=1",
            Graph::from_pairs(5, {{0,1},{0,2},{0,3},{0,4}}));
}

// Linear path: 0-1-2-3-4
// Optimal d* = 1 (orient alternately)
void test_path() {
    run_all("Path P5  |  expected d*=1",
            Graph::from_pairs(5, {{0,1},{1,2},{2,3},{3,4}}));
}

// Complete graph K4: 6 edges, 4 vertices
// Each vertex needs out-degree exactly 3/2 → d* = ceil(6/4) = 2
void test_k4() {
    run_all("Complete Graph K4  |  expected d*=2",
            Graph::from_pairs(4, {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}}));
}

// Complete graph K5: 10 edges, 5 vertices
// d* = ceil(10/5) = 2
void test_k5() {
    run_all("Complete Graph K5  |  expected d*=2",
            Graph::from_pairs(5, {
                {0,1},{0,2},{0,3},{0,4},
                {1,2},{1,3},{1,4},
                {2,3},{2,4},
                {3,4}
            }));
}

// Bipartite K(3,3): 9 edges
// d* = ceil(9/6) = 2  (actually 3, since every vertex has degree 3)
void test_k33() {
    run_all("Bipartite K(3,3)  |  expected d*=2",
            Graph::from_pairs(6, {
                {0,3},{0,4},{0,5},
                {1,3},{1,4},{1,5},
                {2,3},{2,4},{2,5}
            }));
}

// Petersen graph: 10 vertices, 15 edges, 3-regular
// d* = ceil(15/10) = 2
void test_petersen() {
    run_all("Petersen Graph  |  expected d*=2",
            Graph::from_pairs(10, {
                // outer 5-cycle
                {0,1},{1,2},{2,3},{3,4},{4,0},
                // inner pentagram
                {5,7},{7,9},{9,6},{6,8},{8,5},
                // spokes
                {0,5},{1,6},{2,7},{3,8},{4,9}
            }));
}

// Two triangles sharing one edge (diamond / K4 minus one edge)
// 5 edges, 4 vertices — d* = ceil(5/4) = 2
void test_diamond() {
    run_all("Diamond (K4 minus one edge)  |  expected d*=2",
            Graph::from_pairs(4, {{0,1},{0,2},{1,2},{1,3},{2,3}}));
}

// Wheel W5: centre vertex 0 connected to all, outer 4-cycle
// 8 edges, 5 vertices — d* = ceil(8/5) = 2
void test_wheel() {
    run_all("Wheel W5  |  expected d*=2",
            Graph::from_pairs(5, {
                {0,1},{0,2},{0,3},{0,4},   // spokes
                {1,2},{2,3},{3,4},{4,1}    // rim
            }));
}

// Larger random-ish graph: grid 4x4 (16 vertices, 24 edges)
// d* = ceil(24/16) = 2
void test_grid_4x4() {
    vector<pair<int,int>> edges;
    // horizontal edges
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 3; ++c)
            edges.push_back({r*4+c, r*4+c+1});
    // vertical edges
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            edges.push_back({r*4+c, (r+1)*4+c});

    run_all("Grid 4x4  |  expected d*=2",
            Graph::from_pairs(16, edges));
}

// ─────────────────────────────────────────────────────────────────────────────
//  Optional: run on an external graph file
// ─────────────────────────────────────────────────────────────────────────────

void run_file(const string& path) {
    try {
        Graph g = Graph::from_edge_list(path);
        run_all("External graph: " + path, g);
    } catch (const exception& e) {
        cerr << "Error loading " << path << ": " << e.what() << "\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    cout << "\n";
    print_separator('*');
    cout << "  RPO Test Suite — RapidPathOrientation full pipeline\n";
    print_separator('*');
    cout << "\n";

    // ── Hardcoded unit tests ──────────────────────────────────────────────
    test_triangle();
    test_star();
    test_path();
    test_k4();
    test_k5();
    test_k33();
    test_petersen();
    test_diamond();
    test_wheel();
    test_grid_4x4();

    // ── Optional external graph file ─────────────────────────────────────
    if (argc >= 2) {
        for (int i = 1; i < argc; ++i)
            run_file(argv[i]);
    } else {
        print_separator();
        cout << "  All hardcoded tests complete.\n"
             << "  To also run on a graph file:\n"
             << "    ./run  path/to/graph.txt\n"
             << "  File format: one edge per line  \"u v\"  (# = comment)\n";
        print_separator();
    }

    cout << "\n";
    return 0;
}
