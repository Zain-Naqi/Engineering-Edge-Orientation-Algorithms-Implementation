#pragma once
#include "Graph.h"
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <cassert>

using namespace std;

// ── Venkateswaran DFS variant ─────────────────────────────────────────────────
//
//  Same logic as Venkateswaran.h but uses DFS instead of BFS for path search.
//  Struct renamed VenkatDFSResult to avoid ODR conflict when both headers
//  are included in the same translation unit.

struct VenkatDFSResult {
    int       d_star;
    long long paths_flipped;
    long long edges_visited;
};

static vector<int> dfs_find_path_venkat(
    const Graph&              g,
    const OrientationState&   s,
    const vector<int>&        sources,
    const unordered_set<int>& T_set,
    vector<int>&              parent_edge,
    vector<int>&              parent_vtx,
    long long&                edges_visited
) {
    vector<int> stk;
    vector<int> visited_list;

    for (int src : sources) {
        if (parent_edge[src] == -1) {
            parent_edge[src] = -2;
            parent_vtx[src]  = -2;
            stk.push_back(src);
            visited_list.push_back(src);
        }
    }

    int sink_found = -1;

    while (!stk.empty() && sink_found == -1) {
        int u = stk.back();
        stk.pop_back();

        for (int eid : g.incident(u)) {
            edges_visited++;
            if (!s.is_outgoing(eid, u, g.edges())) continue;

            int v = s.other_endpoint(eid, u, g.edges());
            if (parent_edge[v] != -1) continue;

            parent_edge[v] = eid;
            parent_vtx[v]  = u;
            visited_list.push_back(v);

            if (T_set.count(v)) { sink_found = v; break; }
            stk.push_back(v);
        }
    }

    vector<int> path;
    if (sink_found != -1) {
        int cur = sink_found;
        while (parent_vtx[cur] != -2) {
            path.push_back(parent_edge[cur]);
            cur = parent_vtx[cur];
        }
        reverse(path.begin(), path.end());
    }

    for (int v : visited_list) {
        parent_edge[v] = -1;
        parent_vtx[v]  = -1;
    }

    return path;
}

inline VenkatDFSResult venkateswaran_dfs(const Graph& g, OrientationState& s) {
    VenkatDFSResult result{0, 0, 0};
    const int n = g.n();

    vector<int> parent_edge(n, -1);
    vector<int> parent_vtx(n, -1);

    int k = s.max_outdeg();

    unordered_set<int> S, T;
    for (int v = 0; v < n; ++v) {
        if (s.outdeg[v] == k)      S.insert(v);
        if (s.outdeg[v] <= k - 2)  T.insert(v);
    }

    while (true) {
        if (S.empty()) {
            k--;
            if (k <= 0) break;
            S.clear(); T.clear();
            for (int v = 0; v < n; ++v) {
                if (s.outdeg[v] == k)      S.insert(v);
                if (s.outdeg[v] <= k - 2)  T.insert(v);
            }
            continue;
        }

        vector<int> S_vec(S.begin(), S.end());
        vector<int> path = dfs_find_path_venkat(
            g, s, S_vec, T, parent_edge, parent_vtx, result.edges_visited);

        if (path.empty()) break;

        int src_vtx  = s.source(path.front(), g.edges());
        int sink_vtx = s.dest(path.back(),  g.edges());

        s.flip_path(path, g.edges());
        result.paths_flipped++;

        S.erase(src_vtx);
        if (s.outdeg[src_vtx] <= k - 2)  T.insert(src_vtx);

        T.erase(sink_vtx);
        if (s.outdeg[sink_vtx] == k)      S.insert(sink_vtx);
    }

    result.d_star = k;
    assert(s.verify(g.edges()));
    return result;
}
