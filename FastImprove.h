#pragma once
#include "Graph.h"
#include <cassert>

// ── FastImprove (Algorithm 3 in the paper) ────────────────────────────────
//
//  One O(m) pass over every edge.  For each edge v→u, if flipping it
//  (making it u→v) would strictly reduce the larger of the two out-degrees,
//  flip it.  Only direct single-edge improvements are made — no path search.
//
//  This yields a much better starting orientation for EPS / optimized_dfs,
//  dramatically cutting the number of subsequent path searches needed.

struct FastImproveResult {
    int       initial_max_outdeg;
    int       final_max_outdeg;
    long long edges_flipped;
};

inline FastImproveResult fast_improve(const Graph& g, OrientationState& s) {
    FastImproveResult result;
    result.initial_max_outdeg = s.max_outdeg();
    result.edges_flipped      = 0;

    for (int eid = 0; eid < g.m(); ++eid) {
        int src = s.source(eid, g.edges());
        int dst = s.dest(eid, g.edges());

        // Flip src→dst to dst→src if dst is sufficiently less loaded
        if (s.outdeg[dst] < s.outdeg[src] - 1) {
            s.flip(eid, g.edges());
            result.edges_flipped++;
        }
    }

    result.final_max_outdeg = s.max_outdeg();
    assert(s.verify(g.edges()));
    return result;
}
