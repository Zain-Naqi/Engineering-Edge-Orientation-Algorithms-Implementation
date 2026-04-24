#pragma once
#include "Graph.h"

struct FastImproveResult {
    int initial_max_outdeg;
    int final_max_outdeg;
    long long edges_flipped;
};

inline FastImproveResult fast_improve(const Graph& g, OrientationState& s) {
    FastImproveResult result;
    result.initial_max_outdeg = s.max_outdeg();
    result.edges_flipped = 0;

    // iterate over every edge by ID
    for (int eid = 0; eid < g.m(); ++eid) {

        // find current source and destination of this edge
        int src = s.source(eid, g.edges());
        int dst = s.dest(eid, g.edges());

        // if dst is underloaded relative to src, flipping helps
        if (s.outdeg[dst] < s.outdeg[src] - 1) {
            s.flip(eid, g.edges());
            result.edges_flipped++;
        }
    }

    result.final_max_outdeg = s.max_outdeg();
    assert(s.verify(g.edges()));
    return result;
}