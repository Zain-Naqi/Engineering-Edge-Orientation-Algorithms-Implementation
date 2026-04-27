#pragma once
#include "Graph.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <cassert>

using namespace std;

// ── Batched BFS (paper §4.2.3) ────────────────────────────────────────────────
//
//  Instead of starting a separate BFS from each peak vertex, ALL peak
//  vertices are inserted into the queue simultaneously at the start.
//  The BFS then fans out and flips the first improving path found for any
//  peak, then continues until all peaks are covered or the queue is exhausted.
//
//  Used by RPO as the cleanup pass after EPS when d* >= 10.
//  (OptimizedDFS is used when d* < 10 — see paper §5.1 and EPS.h.)

struct BFSResult {
    int       d_star;
    long long paths_flipped;
    long long edges_visited;
};

inline BFSResult batched_bfs(const Graph& g, OrientationState& s) {
    BFSResult result{0, 0, 0};
    const int n = g.n();

    vector<int>  parent_edge(n, -1);
    vector<int>  parent_vtx(n, -1);

    bool improved = true;
    while (improved) {
        improved = false;

        int k = s.max_outdeg();
        if (k <= 0) break;

        // Collect all current peak vertices
        vector<int> peaks;
        peaks.reserve(n / 4);
        for (int v = 0; v < n; ++v)
            if (s.outdeg[v] == k) peaks.push_back(v);
        if (peaks.empty()) break;

        // BFS from ALL peaks simultaneously
        queue<int> q;
        vector<int> visited_list;

        for (int src : peaks) {
            if (parent_edge[src] == -1) {
                parent_edge[src] = -2;
                parent_vtx[src]  = -2;
                q.push(src);
                visited_list.push_back(src);
            }
        }

        // Map: peak that "owns" a discovered vertex (for path reconstruction)
        // We track which original peak vertex each BFS tree is rooted at
        // so we can attribute a found sink back to the correct peak.
        vector<int> root_of(n, -1);
        for (int src : peaks) root_of[src] = src;

        // Process BFS; collect (sink, root) pairs found
        vector<pair<int,int>> found_sinks;   // (sink_vertex, root_peak)

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (int eid : g.incident(u)) {
                result.edges_visited++;
                if (!s.is_outgoing(eid, u, g.edges())) continue;

                int v = s.other_endpoint(eid, u, g.edges());
                if (parent_edge[v] != -1) continue;

                parent_edge[v] = eid;
                parent_vtx[v]  = u;
                root_of[v]     = root_of[u];
                visited_list.push_back(v);

                if (s.outdeg[v] <= k - 2) {
                    found_sinks.push_back({v, root_of[v]});
                    // Don't break — keep going so other peaks may find paths
                }

                q.push(v);
            }
        }

        // Flip paths for each found (sink, peak) pair where the peak is still
        // at out-degree k (a previous flip in this batch may have already
        // improved it).
        for (auto [sink, root] : found_sinks) {
            if (s.outdeg[root] != k) continue;   // already improved

            // Reconstruct path from root to sink
            vector<int> path;
            int cur = sink;
            while (parent_vtx[cur] != -2) {
                path.push_back(parent_edge[cur]);
                cur = parent_vtx[cur];
            }
            reverse(path.begin(), path.end());

            if (!path.empty()) {
                s.flip_path(path, g.edges());
                result.paths_flipped++;
                improved = true;
            }
        }

        // Reset scratch buffers (only touched entries)
        for (int v : visited_list) {
            parent_edge[v] = -1;
            parent_vtx[v]  = -1;
            root_of[v]     = -1;
        }
    }

    result.d_star = s.max_outdeg();
    assert(s.verify(g.edges()));
    return result;
}
