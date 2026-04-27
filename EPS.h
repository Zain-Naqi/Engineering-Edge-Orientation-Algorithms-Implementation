#pragma once
#include "Graph.h"
#include "OptimizedDFS.h"
#include "BatchedBFS.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>

using namespace std;

// ── Eager Path Search (EPS, paper §4.2.3 + §5.1) ────────────────────────────
//
//  Rather than only searching from peak (max out-degree) vertices, EPS first
//  processes the outer "layers" of the onion structure — vertices whose
//  out-degree is slightly below the maximum.  This avoids repeated long-path
//  searches from the peak layer through many intermediate layers.
//
//  Parameters (tuned in paper §5.1):
//    i  = number of outer layers processed eagerly
//         = sqrt(maxd - rho)   [dynamic, best on average]
//    c  = eager-size threshold = 100
//         If a layer has fewer than c vertices, keep searching it
//         repeatedly (without re-collecting) until exhausted.
//
//  After EPS a cleanup pass corrects any paths blocked by the shared visited
//  array.  Paper §5.1: use optimized DFS when d* < 10, batched BFS otherwise.
//
//  Bug fixed from original EPS.h:
//    dfs_find_path received parent_edge BY VALUE — writes were silently lost,
//    breaking the shared-array trick.  Now passed by reference (matching the
//    corrected OptimizedDFS.h signature).

struct EPSResult {
    int       d_star;
    long long paths_flipped;
    long long edges_visited;
};

// ── Single-layer DFS pass ────────────────────────────────────────────────────
//  Iterates over layer_vertices and tries to find an improving path for each
//  vertex currently at out-degree `layer_deg`.
//  visited/parent arrays are shared across calls (not reset here).

static long long eps_layer_pass(
    const Graph&      g,
    OrientationState& s,
    int               layer_deg,
    const vector<int>& layer_vertices,
    vector<bool>&     visited,
    vector<int>&      parent_edge,   // FIX: reference (was value copy in original)
    vector<int>&      parent_vtx,
    long long&        edges_visited
) {
    long long paths_flipped = 0;

    for (int v : layer_vertices) {
        if (s.outdeg[v] != layer_deg) continue;   // already improved

        vector<int> path = dfs_find_path(
            g, s, v, layer_deg,
            visited, parent_edge, parent_vtx,
            edges_visited);

        if (!path.empty()) {
            s.flip_path(path, g.edges());
            paths_flipped++;
        }
    }
    return paths_flipped;
}

inline EPSResult eps(const Graph& g, OrientationState& s) {
    EPSResult result{0, 0, 0};
    const int n = g.n();

    // ── EPS parameters ────────────────────────────────────────────────────
    int    maxd        = s.max_outdeg();
    double rho         = g.density();
    int    i           = max(1, (int)sqrt((double)max(0, maxd - (int)rho)));
    const int c        = 100;

    int start_layer = max(1, maxd - i);

    // ── Shared scratch buffers ────────────────────────────────────────────
    vector<bool> visited(n, false);
    vector<int>  parent_edge(n, -1);
    vector<int>  parent_vtx(n, -1);

    // ── Process layers outer → inner (start_layer … maxd) ────────────────
    for (int layer = start_layer; layer <= maxd; ++layer) {

        // Collect vertices currently at this out-degree level
        vector<int> layer_vertices;
        layer_vertices.reserve(64);
        for (int v = 0; v < n; ++v)
            if (s.outdeg[v] == layer) layer_vertices.push_back(v);

        // if (layer_vertices.empty()) continue;

        // Reset visited fresh for each new layer
        // fill(visited.begin(),     visited.end(),     false);
        // fill(parent_edge.begin(), parent_edge.end(), -1);
        // fill(parent_vtx.begin(),  parent_vtx.end(),  -1);

        if ((int)layer_vertices.size() < c) {
            // ── Eager mode: small layer — keep searching until exhausted ──
            bool improved = true;
            while (improved) {
                long long flipped = eps_layer_pass(
                    g, s, layer, layer_vertices,
                    visited, parent_edge, parent_vtx,
                    result.edges_visited);
                result.paths_flipped += flipped;
                improved = (flipped > 0);

                if (flipped > 0) {
                improved = true;
                layer_vertices.clear();
                for (int v = 0; v < n; ++v)
                    if (s.outdeg[v] == layer) layer_vertices.push_back(v);
                if (layer_vertices.empty()) break;
                }

                // Reset arrays between eager sub-passes within the same layer
                // fill(visited.begin(),     visited.end(),     false);
                // fill(parent_edge.begin(), parent_edge.end(), -1);
                // fill(parent_vtx.begin(),  parent_vtx.end(),  -1);
            }
        } else {
            // ── Normal mode: single pass with shared visited array ─────────
            long long flipped = eps_layer_pass(
                g, s, layer, layer_vertices,
                visited, parent_edge, parent_vtx,
                result.edges_visited);
            result.paths_flipped += flipped;
        }
    }

    // ── Cleanup pass ──────────────────────────────────────────────────────
    //  EPS may block paths via the shared visited array.
    //  Paper §5.1: use optimized DFS when d* < 10, batched BFS otherwise.
    int d_after_eps = s.max_outdeg();
    if (d_after_eps < 10) {
        DFSResult cleanup = optimized_dfs(g, s);
        result.paths_flipped += cleanup.paths_flipped;
        result.edges_visited += cleanup.edges_visited;
    } else {
        BFSResult cleanup = batched_bfs(g, s);
        result.paths_flipped += cleanup.paths_flipped;
        result.edges_visited += cleanup.edges_visited;
    }

    result.d_star = s.max_outdeg();
    assert(s.verify(g.edges()));
    return result;
}
