#pragma once
#include "Graph.h"
#include "TwoApprox.h"
#include "FastImprove.h"
#include "EPS.h"

// ── RapidPathOrientation (RPO) — paper §5.1 ──────────────────────────────────
//
//  This is the complete final algorithm from the paper.  It wires every
//  engineered component together in the order described in §5.1:
//
//  Step 1  [Conditional]  2-Approximation data reduction
//                         Run only when average density rho = m/n > 10.
//                         (Running it on sparse graphs wastes more time than
//                          it saves — paper §5.1 "Two Approximation" paragraph.)
//
//  Step 2                 FastImprove
//                         One O(m) pass of single-edge direct improvements.
//                         Always run — dramatically reduces the starting d*
//                         and shortens the paths EPS needs to find.
//
//  Step 3                 Eager Path Search (EPS)
//                         Processes outer layers eagerly with dynamic layer
//                         count i = sqrt(maxd - rho) and eager-size c = 100.
//                         EPS internally calls the correct cleanup pass:
//                           - optimized DFS  when d* < 10
//                           - batched BFS    when d* >= 10
//
//  All stats are accumulated and returned in RPOResult.

struct RPOResult {
    int       d_star;

    // Per-stage stats
    int       d_after_two_approx;    // -1 if skipped
    int       d_after_fast_improve;
    int       d_after_eps;

    long long two_approx_flipped;    // 0 if skipped
    long long fast_improve_flipped;
    long long eps_paths_flipped;
    long long eps_edges_visited;
};

inline RPOResult rpo(const Graph& g, OrientationState& s) {
    RPOResult r{};
    r.d_after_two_approx = -1;
    r.two_approx_flipped = 0;

    // ── Step 1: Conditional 2-approximation ──────────────────────────────
    if (g.density() > 10.0) {
        TwoApproxResult ta = two_approx(g, s);
        r.d_after_two_approx  = ta.d_approx;
        r.two_approx_flipped  = ta.edges_flipped;
    }

    // ── Step 2: FastImprove ───────────────────────────────────────────────
    FastImproveResult fi = fast_improve(g, s);
    r.d_after_fast_improve = fi.final_max_outdeg;
    r.fast_improve_flipped = fi.edges_flipped;

    // ── Step 3: EPS (with internal cleanup pass) ──────────────────────────
    EPSResult ep = eps(g, s);
    r.d_after_eps      = ep.d_star;
    r.eps_paths_flipped = ep.paths_flipped;
    r.eps_edges_visited = ep.edges_visited;

    r.d_star = ep.d_star;
    return r;
}
