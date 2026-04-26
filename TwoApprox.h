#pragma once
#include "Graph.h"
#include <vector>
#include <algorithm>
#include <cassert>
#include <iostream>

using namespace std;

//  ── 2-Approximation via Min-Degree Peeling ───────────────────────────────
//
//  Algorithm (Charikar [10], Section 4.2.1 of the paper):
//
//    Repeatedly remove the minimum-degree vertex from the graph (by
//    undirected degree), orienting all its edges OUTWARD at the moment of
//    removal.  Track the maximum undirected degree seen at any removal.
//    That maximum is d_approx, a 2-approximation of the optimal d*.
//
//  Why it's a 2-approx:
//    The optimal d* equals the ceiling of the maximum subgraph density
//    (edges/vertices).  In the densest subgraph every vertex has degree
//    >= d* within that subgraph, so no vertex there is peeled before the
//    rest of the graph is gone.  The last vertex peeled from the densest
//    subgraph had degree at most 2*d* (each of its d* edges touches one
//    other endpoint), giving d_approx <= 2*d*, i.e. d* >= d_approx/2.
//
//  Data-reduction guarantee:
//    Because d* >= d_approx/2, any vertex whose undirected degree at
//    removal time was <= floor(d_approx/2) will have out-degree <=
//    floor(d_approx/2) <= d* after we orient its edges outward.  It can
//    therefore never be a peak vertex in the optimal solution — we never
//    need to touch it again.  Subsequent path-finding algorithms naturally
//    skip these vertices because they are not peaks.
//
//  Conditional use (paper Section 4.2.1):
//    Running the 2-approx on a sparse graph wastes time because the bucket
//    queue and neighbour-degree updates cost O(m) even when d_approx turns
//    out to be tiny and no vertices are actually removed from further
//    consideration.  The paper recommends running it only when
//    average density rho = m/n > 10.  The caller is responsible for this
//    check (see Run_OptimizedDFS.cpp / RPO entry point).
//
//  What this function does to OrientationState:
//    It modifies s in-place: every edge is reoriented outward from the
//    vertex that was peeled first (the lower-degree endpoint at removal
//    time).  On return, s holds a valid complete orientation whose
//    max-outdegree is d_approx.  Subsequent algorithms (FastImprove, EPS,
//    optimized_dfs) will further improve this to d*.
//
//  ─────────────────────────────────────────────────────────────────────────

struct TwoApproxResult {
    int  d_approx;          // 2-approximation of optimal max out-degree
    int  safe_removed;      // # vertices with peel-degree <= d_approx/2
                            //   (these will never be peaks; no further work needed)
    long long edges_flipped;    // edges actually flipped during peeling
};


inline TwoApproxResult two_approx(const Graph& g, OrientationState& s) {

    const int n = g.n();

    // ── Step 1: compute initial undirected degrees ────────────────────────
    //
    //  We maintain a SEPARATE degree array that counts undirected edges in
    //  the "remaining" graph (i.e. edges to not-yet-removed vertices).
    //  This is NOT s.outdeg — that tracks directed out-degree and will be
    //  updated by s.flip() as we orient edges.

    vector<int> deg(n);
    for (int v = 0; v < n; ++v)
        deg[v] = (int)g.incident(v).size();

    int max_deg = (n > 0) ? *max_element(deg.begin(), deg.end()) : 0;

    // ── Step 2: initialise bucket priority queue ──────────────────────────
    //
    //  buckets[d] holds vertices whose CURRENT remaining-graph degree is d.
    //  We use lazy deletion: when a vertex's degree drops, we push it into
    //  its new bucket but leave the stale entry in the old bucket.  When we
    //  pop a vertex from a bucket, we validate it before processing.

    vector<vector<int>> buckets(max_deg + 1);
    for (int v = 0; v < n; ++v)
        buckets[deg[v]].push_back(v);

    vector<bool> removed(n, false);

    int d_approx     = 0;
    int safe_removed = 0;
    long long edges_flipped = 0;
    int min_bucket   = 0;

    // ── Step 3: peel all n vertices in min-degree order ───────────────────

    for (int processed = 0; processed < n; ) {

        // Advance min_bucket pointer to the first non-empty bucket
        while (min_bucket <= max_deg && buckets[min_bucket].empty())
            ++min_bucket;
        if (min_bucket > max_deg) break;

        // Lazy-deletion pop: discard stale entries until we get a live vertex
        int v = -1;
        while (!buckets[min_bucket].empty()) {
            int cand = buckets[min_bucket].back();
            buckets[min_bucket].pop_back();

            // A stale entry exists if deg[cand] no longer matches the bucket
            // it was pushed into, OR if it was already removed.
            if (!removed[cand] && deg[cand] == min_bucket) {
                v = cand;
                break;
            }
        }
        if (v == -1) continue;   // entire bucket was stale; outer loop will advance

        // ── Remove v ─────────────────────────────────────────────────────
        removed[v] = true;
        ++processed;

        //  deg[v] is its degree in the remaining graph at the moment of
        //  removal — this is the value that contributes to d_approx.
        d_approx = max(d_approx, deg[v]);

        // ── Orient all edges of v OUTWARD and update neighbour degrees ────
        for (int eid : g.incident(v)) {

            //  other_endpoint is direction-independent; it always returns
            //  the vertex on the other side of the edge from v.
            int u = s.other_endpoint(eid, v, g.edges());

            //  Make v → u.  If the edge is already outgoing from v, do
            //  nothing.  Otherwise flip it.
            if (!s.is_outgoing(eid, v, g.edges())) {
                s.flip(eid, g.edges());
                ++edges_flipped;
            }

            //  Update u's remaining-graph degree only if u is still alive.
            if (!removed[u]) {
                --deg[u];

                //  Push u into its new (lower) bucket.
                //  The old entry at deg[u]+1 is now stale and will be
                //  discarded by lazy deletion when we encounter it.
                buckets[deg[u]].push_back(u);

                //  min_bucket must never overshoot a newly created lower
                //  bucket, so clamp it downward if needed.
                min_bucket = min(min_bucket, deg[u]);
            }
        }
    }

    // ── Step 4: count how many vertices were safely eliminated ────────────
    //
    //  A vertex is "safe" if its degree at peeling time was <= d_approx/2.
    //  Because d* >= d_approx/2, such vertices have out-degree <= d* and
    //  will never appear as peaks in subsequent path-finding.
    //
    //  We recompute this now that d_approx is known.  We can't count during
    //  the loop because d_approx is only finalised at the end.

    int threshold = d_approx / 2;   // integer floor

    //  We need to know each vertex's degree-at-removal.  We don't store it
    //  explicitly above to keep memory lean; instead we check s.outdeg[v]
    //  which, after the peeling loop, equals exactly the peel-degree of v
    //  (since we orient all edges outward, out-degree == undirected degree
    //  at removal time for every vertex).

    for (int v = 0; v < n; ++v) {
        if (s.outdeg[v] <= threshold)
            ++safe_removed;
    }

    assert(s.verify(g.edges()));

    return { d_approx, safe_removed, edges_flipped };
}
