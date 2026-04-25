#pragma once
#include "Graph.h"
#include <vector>
#include <stack>
#include <algorithm>
#include <cassert>
#include <iostream>
#include <stdio.h>


using namespace std;


struct DFSResult {
    int d_star;
    long long paths_flipped;
    long long edges_visited;
};


// Optimized DFS with Shared Visited Array.



static vector<int> dfs_find_path(
    const Graph& g,
    const OrientationState& s,
    int start,
    int k,
    vector<bool>& visited,
    vector<int> parent_edge,
    vector<int>& parent_vtx,
    long long& edges_visited
) {
    stack<int> stk;

    if (visited[start]) return {};

    visited[start] = true;
    parent_edge[start] = -2;
    parent_vtx[start] = -2;
    stk.push(start);

    int sink_found = -1;

    while (!stk.empty() && sink_found == -1) {
        int u = stk.top();
        stk.pop();

        for (int eid : g.incident(u)) {
            if (!s.is_outgoing(eid, u, g.edges())) continue;

            int v = s.other_endpoint(eid, u, g.edges());
            edges_visited++;

            // Improvement # 01: Check if v is already a sink before any further exploration
            if (s.outdeg[v] <= k - 2) {
                parent_edge[v] = eid;
                parent_vtx[v] = u;
                sink_found = v;
                break;
            }

            // Improvement # 02: Do not traverse through other peak vertices
            if (s.outdeg[v] == k) continue;

            // Improvement # 03: Shared Visited Array - Skip vertices if already explored in this pass
            if (visited[v]) continue;

            visited[v] = true;
            parent_edge[v] = eid;
            parent_vtx[v] = u;
            stk.push(v);
        }
    }

    if (sink_found == -1) return {};

    vector<int> path;
    int cur = sink_found;
    while (parent_vtx[cur] != -2) {
        path.push_back(parent_edge[cur]);
        cur = parent_vtx[cur];
    }

    std::reverse(path.begin(), path.end());
    return path;
}


inline DFSResult optimized_dfs(const Graph& g, OrientationState& s) {
    
    DFSResult result{0, 0, 0};
    const int n = g.n();

    vector<bool> visited(n, false);
    vector<int> parent_edge(n, -1);
    vector<int> parent_vtx(n, -1);

    bool improved = true;

    while (improved) {
        improved = false;
 
        // Start of new pass: reset visited[]
        std::fill(visited.begin(), visited.end(), false);
 
        int k = s.max_outdeg();
        if (k <= 0) break;
 
        // Collect peak vertices before the pass.
        // We snapshot them so flips mid-pass don't change who we process.
        std::vector<int> peaks;
        peaks.reserve(n / 4);
        for (int v = 0; v < n; ++v)
            if (s.outdeg[v] == k) peaks.push_back(v);
 
        for (int v : peaks) {
            // A previous flip in this pass may have reduced v's out-degree.
            // If v is no longer a peak, skip it.
            if (s.outdeg[v] != k) continue;
 
            std::vector<int> path = dfs_find_path(
                g, s, v, k,
                visited, parent_edge, parent_vtx,
                result.edges_visited
            );
 
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

