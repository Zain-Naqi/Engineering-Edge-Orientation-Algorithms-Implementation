#pragma once
#include "Graph.h"
#include "OptimizedDFS.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std;

//  Eager Path Search (EPS)

//
//  Parameters:
//    i  = number of layers to eagerly reduce = sqrt(maxd - rho)  [dynamic]
//    c  = eager size threshold = 100
//         if a layer has fewer than c vertices, search it repeatedly
//         without re-collecting vertices until no more improvements found
//
//  After EPS, a final cleanup pass of optimized_dfs is run to catch any
//  paths that EPS may have blocked via the shared visited array.

struct EPSResult {
    int d_star;
    long long paths_flipped;
    long long edges_visited;
};

//  Single-layer DFS pass 
//
//  Searches for improving paths starting only from vertices at out-degree
//  exactly `layer_deg`. A valid endpoint is any vertex with out-degree
//  <= layer_deg - 2.
//
//  Uses the shared visited array from the caller — NOT reset between
//  individual vertex searches within a layer (that's the shared array trick).
//  The caller is responsible for resetting visited between layers.
//
//  Returns number of paths flipped in this pass.

static long long eps_layer_pass(
    const Graph& g,
    OrientationState& s,
    int layer_deg,
    const vector<int>& layer_vertices,
    vector<bool>& visited,
    vector<int>& parent_edge,
    vector<int>& parent_vtx,
    long long& edges_visited
) {
    long long paths_flipped = 0;

    for (int v : layer_vertices) {
        // vertex may have been improved by a previous flip in this pass
        if (s.outdeg[v] != layer_deg) continue;

        vector<int> path = dfs_find_path(
            g, s, v, layer_deg,
            visited, parent_edge, parent_vtx,
            edges_visited
        );

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

    // ── Compute EPS parameters ────────────────────────────────────────────
    int maxd        = s.max_outdeg();
    double rho      = g.density();
    int i           = max(1, (int)sqrt((double)(maxd - rho)));
    const int c     = 100;          // eager size threshold

    // layers to process eagerly, from outermost to innermost
    // e.g. if maxd=6, i=2: process layers 4, 5, 6  (outer to core)
    int start_layer = max(1, maxd - i);

    // ── Shared scratch buffers (same pattern as OptimizedDFS) ─────────────
    vector<bool> visited(n, false);
    vector<int>  parent_edge(n, -1);
    vector<int>  parent_vtx(n, -1);

    // ── Process layers outer → inner ─────────────────────────────────────
    for (int layer = start_layer; layer <= maxd; layer++) {

        // Collect vertices currently at this layer's out-degree
        vector<int> layer_vertices;
        layer_vertices.reserve(n / 4);
        for (int v = 0; v < n; ++v) {
            if (s.outdeg[v] == layer) layer_vertices.push_back(v);
        }

        if (layer_vertices.empty()) continue;

        // Reset visited array fresh for each new layer
        fill(visited.begin(), visited.end(), false);

        if ((int)layer_vertices.size() < c) {
            // ── Eager mode: small layer ───────────────────────────────────
            // Keep searching this layer repeatedly until no improvements
            // found, without re-collecting vertices each time.
            // This avoids the overhead of rebuilding layer_vertices when
            // only a handful of vertices are in this layer.
            bool improved = true;
            while (improved) {
                long long flipped = eps_layer_pass(
                    g, s, layer,
                    layer_vertices,
                    visited, parent_edge, parent_vtx,
                    result.edges_visited
                );
                result.paths_flipped += flipped;
                improved = (flipped > 0);

                // Reset visited between eager sub-passes within the same layer
                fill(visited.begin(), visited.end(), false);
            }
        } else {
            // ── Normal mode: large layer ──────────────────────────────────
            // Single pass over the layer with shared visited array.
            long long flipped = eps_layer_pass(
                g, s, layer,
                layer_vertices,
                visited, parent_edge, parent_vtx,
                result.edges_visited
            );
            result.paths_flipped += flipped;
        }
    }

    // ── Final cleanup pass ────────────────────────────────────────────────
    //
    //  EPS may have blocked valid paths by marking vertices visited in the
    //  shared array. For example: a d-1 layer vertex gets marked as visited
    //  and no improving path found from it, but there EXISTS a path from a
    //  d-layer vertex through it to a d-2 vertex. EPS misses this.
    //
    //  A full optimized_dfs pass catches everything EPS left behind.
    //  (Later: swap for batched BFS when d* >= 10)

    DFSResult cleanup = optimized_dfs(g, s);
    result.paths_flipped += cleanup.paths_flipped;
    result.edges_visited += cleanup.edges_visited;

    result.d_star = s.max_outdeg();
    assert(s.verify(g.edges()));
    return result;
}