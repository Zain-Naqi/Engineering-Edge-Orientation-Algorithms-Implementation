#pragma once
#include "Graph.h"
#include <vector>
#include <algorithm>
#include <cassert>

using namespace std;

// ── 2-Approximation via Min-Degree Peeling (Charikar, Section 4.2.1) ────────
//
//  Repeatedly remove the minimum-degree vertex from the remaining graph,
//  orienting all its edges OUTWARD at the moment of removal.
//  The maximum undirected degree seen at any removal is d_approx — a
//  2-approximation of the optimal d*.
//
//  Conditional use (paper §4.2.1):
//    Only beneficial when average density rho = m/n > 10.
//    The RPO entry point enforces this check.

struct TwoApproxResult {
    int       d_approx;       // 2-approximation of optimal max out-degree
    int       safe_removed;   // vertices with peel-degree <= d_approx/2
    long long edges_flipped;
};

inline TwoApproxResult two_approx(const Graph& g, OrientationState& s) {
    const int n = g.n();

    // Remaining-graph undirected degree (separate from s.outdeg)
    vector<int> deg(n);
    for (int v = 0; v < n; ++v)
        deg[v] = (int)g.incident(v).size();

    int max_deg = (n > 0) ? *max_element(deg.begin(), deg.end()) : 0;

    // Bucket priority queue with lazy deletion
    vector<vector<int>> buckets(max_deg + 1);
    for (int v = 0; v < n; ++v)
        buckets[deg[v]].push_back(v);

    vector<bool> removed(n, false);
    int d_approx     = 0;
    int safe_removed = 0;
    long long edges_flipped = 0;
    int min_bucket   = 0;

    for (int processed = 0; processed < n; ) {
        // Advance to first non-empty bucket
        while (min_bucket <= max_deg && buckets[min_bucket].empty())
            ++min_bucket;
        if (min_bucket > max_deg) break;

        // Lazy-deletion pop
        int v = -1;
        while (!buckets[min_bucket].empty()) {
            int cand = buckets[min_bucket].back();
            buckets[min_bucket].pop_back();
            if (!removed[cand] && deg[cand] == min_bucket) {
                v = cand;
                break;
            }
        }
        if (v == -1) continue;

        removed[v] = true;
        ++processed;
        d_approx = max(d_approx, deg[v]);

        // Orient all edges of v outward; update neighbour degrees
        for (int eid : g.incident(v)) {
            int u = s.other_endpoint(eid, v, g.edges());
            if (!s.is_outgoing(eid, v, g.edges())) {
                s.flip(eid, g.edges());
                ++edges_flipped;
            }
            if (!removed[u]) {
                --deg[u];
                buckets[deg[u]].push_back(u);
                min_bucket = min(min_bucket, deg[u]);
            }
        }
    }

    // Count safely eliminated vertices (out-degree <= floor(d_approx/2))
    int threshold = d_approx / 2;
    for (int v = 0; v < n; ++v)
        if (s.outdeg[v] <= threshold) ++safe_removed;

    assert(s.verify(g.edges()));
    return { d_approx, safe_removed, edges_flipped };
}
