#pragma once
#include "Graph.h"
#include <vector>
#include <stack>
#include <algorithm>
#include <cassert>

using namespace std;

// ── Optimized DFS with Shared Visited Array ───────────────────────────────────
//
//  Three engineering improvements over naive DFS (paper §4.2.3):
//
//  1. Early Check  — inspect out-degree of each neighbour before recursing,
//                    short-circuiting when a sink is found immediately.
//  2. Independent Paths — skip peak vertices (outdeg == k) during traversal;
//                         each peak needs its own independent improving path.
//  3. Shared Visited Array — visited[] is NOT reset between consecutive
//                            searches within a single pass over peak vertices.
//                            Once a sub-graph is proven non-improving it is
//                            excluded from all subsequent searches in that pass.
//
//  Bug fixed from original:
//    parent_edge was passed BY VALUE in dfs_find_path, so writes were
//    discarded after each call — the shared-array trick was silently broken.
//    Changed to pass by reference.

struct DFSResult {
    int       d_star;
    long long paths_flipped;
    long long edges_visited;
};

// ── Core path-finder ─────────────────────────────────────────────────────────
//
//  Finds one improving path starting at `start` using the current orientation.
//  A valid sink is any vertex with outdeg <= k-2.
//
//  visited[], parent_edge[], parent_vtx[] are shared across calls within a
//  pass — callers must NOT reset them between individual vertex searches.
//  Callers reset them between full passes over peak vertices.
//
//  Returns the path as an ordered list of edge IDs (source → sink).
//  Returns an empty vector if no improving path exists from `start`.

static vector<int> dfs_find_path(
    const Graph&      g,
    const OrientationState& s,
    int               start,
    int               k,
    vector<bool>&     visited,      // shared — NOT reset between calls
    vector<int>&      parent_edge,  // FIX: was passed by value (copy) → now ref
    vector<int>&      parent_vtx,
    long long&        edges_visited
) {
    if (visited[start]) return {};

    stack<int> stk;
    visited[start]     = true;
    parent_edge[start] = -2;   // sentinel: "I am the root"
    parent_vtx[start]  = -2;
    stk.push(start);

    int sink_found = -1;

    while (!stk.empty() && sink_found == -1) {
        int u = stk.top();
        stk.pop();

        for (int eid : g.incident(u)) {
            if (!s.is_outgoing(eid, u, g.edges())) continue;

            int v = s.other_endpoint(eid, u, g.edges());
            edges_visited++;

            // Improvement 1: early sink check — short-circuit before pushing v
            if (s.outdeg[v] <= k - 2) {
                parent_edge[v] = eid;
                parent_vtx[v]  = u;
                sink_found = v;
                break;
            }

            // Improvement 2: skip other peak vertices
            if (s.outdeg[v] == k) continue;

            // Improvement 3: shared visited array
            if (visited[v]) continue;

            visited[v]     = true;
            parent_edge[v] = eid;
            parent_vtx[v]  = u;
            stk.push(v);
        }
    }

    if (sink_found == -1) return {};

    // Reconstruct path by walking parent pointers
    vector<int> path;
    int cur = sink_found;
    while (parent_vtx[cur] != -2) {
        path.push_back(parent_edge[cur]);
        cur = parent_vtx[cur];
    }
    reverse(path.begin(), path.end());
    return path;
}

// ── Full optimized-DFS pass ───────────────────────────────────────────────────
//
//  Outer loop: keep making passes until no improving path is found in a full
//  pass.  Each pass resets visited[] once, then iterates over all current peak
//  vertices without resetting between individual searches.

inline DFSResult optimized_dfs(const Graph& g, OrientationState& s) {
    DFSResult result{0, 0, 0};
    const int n = g.n();

    vector<bool> visited(n, false);
    vector<int>  parent_edge(n, -1);
    vector<int>  parent_vtx(n, -1);

    bool improved = true;
    while (improved) {
        improved = false;

        // Reset shared arrays once per pass
        fill(visited.begin(),     visited.end(),     false);
        fill(parent_edge.begin(), parent_edge.end(), -1);
        fill(parent_vtx.begin(),  parent_vtx.end(),  -1);

        int k = s.max_outdeg();
        if (k <= 0) break;

        // Snapshot peak vertices (flips mid-pass must not affect who we visit)
        vector<int> peaks;
        peaks.reserve(n / 4);
        for (int v = 0; v < n; ++v)
            if (s.outdeg[v] == k) peaks.push_back(v);

        for (int v : peaks) {
            if (s.outdeg[v] != k) continue;   // already improved by earlier flip

            vector<int> path = dfs_find_path(
                g, s, v, k,
                visited, parent_edge, parent_vtx,
                result.edges_visited);

            if (!path.empty()) {
                s.flip_path(path, g.edges());
                result.paths_flipped++;
                improved = true;
            }
        }
    }

    result.d_star = s.max_outdeg();
    assert(s.verify(g.edges()));
    return result;
}
