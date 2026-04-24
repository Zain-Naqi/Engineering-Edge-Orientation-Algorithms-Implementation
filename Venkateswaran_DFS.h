#pragma once
#include "Graph.h"
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <cassert>

using namespace std;

struct VenkatResult {
    int d_star;                 // optimal max out-degree achieved
    long long paths_flipped;    // total number of improving paths found
    long long edges_visited;    // total edges examined during all BFS calls
};


static vector<int> dfs_find_path(
    const Graph& g,
    const OrientationState& s,
    const vector<int>& sources,
    const unordered_set<int>& T_set,
    vector<int>& parent_edge,
    vector<int>& parent_vtx,
    long long& edges_visited
) {
    vector<int> stack;
    vector<int> visited_list;

    // Initialize DFS stack with all sources
    for (int src : sources) {
        if (parent_edge[src] == -1) {
            parent_edge[src] = -2;   // root marker
            parent_vtx[src] = -2;
            stack.push_back(src);
            visited_list.push_back(src);
        }
    }

    int sink_found = -1;

    while (!stack.empty() && sink_found == -1) {
        int u = stack.back();
        stack.pop_back();

        for (int eid : g.incident(u)) {
            edges_visited++;

            // follow only outgoing edges
            if (!s.is_outgoing(eid, u, g.edges()))
                continue;

            int v = s.other_endpoint(eid, u, g.edges());

            if (parent_edge[v] != -1)
                continue;

            parent_edge[v] = eid;
            parent_vtx[v] = u;
            visited_list.push_back(v);

            if (T_set.count(v)) {
                sink_found = v;
                break;
            }

            stack.push_back(v);
        }
    }

    // reconstruct path
    vector<int> path;

    if (sink_found != -1) {
        int cur = sink_found;

        while (parent_vtx[cur] != -2) {
            path.push_back(parent_edge[cur]);
            cur = parent_vtx[cur];
        }

        reverse(path.begin(), path.end());
    }

    // reset scratch buffers
    for (int v : visited_list) {
        parent_edge[v] = -1;
        parent_vtx[v] = -1;
    }

    return path;
}

inline VenkatResult venkateswaran(const Graph& g, OrientationState& s) {
    VenkatResult result{0, 0, 0};
    const int n = g.n();    


    vector<int> parent_edge(n, -1);
    vector<int> parent_vtx(n, -1);

    int k = s.max_outdeg();

    // Build S = {v : outdeg == k},  T = {v : outdeg <= k - 2}
    // We maintain these as unordered_sets so membership is O(1)
    // and insertion/removal are O(1) amortized.
    unordered_set<int> S, T;
    for (int v = 0; v < n; ++v){
        if (s.outdeg[v] == k)   S.insert(v);
        if (s.outdeg[v] <= k - 2)   T.insert(v);
    }

    while (true) {
        if (S.empty()) {
            // All vertices have out-degree <= k - 1 → reduce target
            k--;
            if (k <= 0) break;

            // Rebuild S and T for new k
            S.clear();
            T.clear();

            for (int v = 0; v < n; ++v) {
                if (s.outdeg[v] == k)   S.insert(v);
                if (s.outdeg[v] <= k - 2)   T.insert(v);
            }

            continue;
        }

        // Convert S to a vector for the BFS source list
        vector<int> S_vec(S.begin(), S.end());

        vector<int> path = dfs_find_path(g, s, S_vec, T, parent_edge, parent_vtx,result.edges_visited);
        // No improving path exists → current k is optimal
        if (path.empty()) break;

        // ── Flip the path ────────────────────────────────────────────────
        // Before flipping, note which endpoint is the source (in S) and
        // which is the sink (in T) so we can update S and T correctly.

        int src_vtx = s.source(path.front(), g.edges());
        int sink_vtx = s.dest(path.back(), g.edges());

        s.flip_path(path, g.edges());
        result.paths_flipped++;

        // Update S: source vertex's out-degree just dropped to k-1
        S.erase(src_vtx);
        // Update T: check if source vertex is now a new sink candidate
        if (s.outdeg[src_vtx] <= k - 2) T.insert(src_vtx);

         // Sink vertex's out-degree just increased by 1
        T.erase(sink_vtx);
        // If it now has out-degree k, add it to 
        if (s.outdeg[sink_vtx] == k)    S.insert(sink_vtx);
        // (out-degree can't exceed k: it was <= k-2, path flip adds only 1)
    }

    result.d_star = k;
    assert(s.verify(g.edges()));
    return result;

}






