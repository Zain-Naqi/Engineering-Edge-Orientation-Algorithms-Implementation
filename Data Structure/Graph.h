#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#include <cassert>

struct Edge {
    int u, v;
};

//  Convention: for edge i = {u, v} with u < v,
//      dir[i] == true   →   edge oriented u → v   (u is the source)
//      dir[i] == false  →   edge oriented v → u   (v is the source)


struct OrientationState {
    std::vector<bool> dir;      // one bool per edge
    std::vector<int> outdeg;    // one int per vertex

    int num_edges() const { 
        return (int)dir.size();
    }

    int num_vertices() const {
        return (int)outdeg.size();
    }

    // Is edge i leaving vertex x?
    bool is_outgoing(int edge_id, int x, const std::vector<Edge>& edges) const {
        const Edge& e = edges[edge_id];
        return (x == e.u) == dir[edge_id];
        // x==u and dir==true  → u→v → outgoing from u
        // x==v and dir==false → v→u → outgoing from v
    }

    // Which vertex is on the other end of edge i from x?
    int other_endpoint(int edge_id, int x, const std::vector<Edge>& edges) const {
        const Edge& e = edges[edge_id];
        return (x == e.u) ? e.v : e.u;
    }

    // Source vertex of edge i under current orientation
    int source(int edge_id, const std::vector<Edge>& edges) const {
        const Edge& e = edges[edge_id];
        return dir[edge_id] ? e.u : e.v;
    }

    // Destination vertex of edge i under current orientation
    int dest(int edge_id, const std::vector<Edge>& edges) const {
        const Edge& e = edges[edge_id];
        return dir[edge_id] ? e.v : e.u;
    }

    // Flip edge i.  Updates outdeg of both endpoints atomically.
    void flip(int edge_id, const std::vector<Edge>& edges) {
        const Edge& e = edges[edge_id];
        if (dir[edge_id]) {
            outdeg[e.u]--;
            outdeg[e.v]++;
        } else {
            outdeg[e.u]++;
            outdeg[e.v]--;
        }
        dir[edge_id] = !dir[edge_id];
    }

    // Flip every edge along a path given as an ordered list of edge IDs.
    // Caller is responsible for ensuring the list forms a valid directed path.
    void flip_path(const std::vector<int>& path_edges, const std::vector<Edge>& edges) {
        for (int eid : path_edges) {
            flip(eid, edges);
        }
    }

    // Current maximum out-degree (linear scan — use only outside hot loops)
    int max_outdeg() const {
        int k = 0;
        for (int d : outdeg) {
            k = std::max(k, d);
        }
        return k;
    }


    // ── Sanity check ──────────────────────────────────────────────────────
    //
    //  Verifies:
    //    1. Every edge has exactly one direction assigned.
    //    2. outdeg[v] == number of edges currently leaving v.
    //
    //  Call after every algorithm during testing.  Remove in benchmarks.
 
    bool verify(const std::vector<Edge>& edges) const {
        std::vector<int> recomputed(outdeg.size(), 0);
        for (int i = 0; i < (int)edges.size(); ++i) {
            int src = dir[i] ? edges[i].u : edges[i].v;
            recomputed[src]++;
        }
        return recomputed == outdeg;
    }
};


class Graph {

public:
    
    static Graph from_edge_list(const std::string& path);

    // Build directly from an (u, v) pair list — useful for unit tests.
    static Graph from_pairs(int num_vertices, const std::vector<std::pair<int, int>>& pairs);

    int n() const { return n_; }
    int m() const { return (int)edges_.size(); }
    double density() const { return (double)m() / n(); }

    const Edge& edge(int i) const { return edges_[i]; }
    const std::vector<Edge>& edges() const { return edges_; }

    // All edge IDs incident to vertex v (undirected)
    const std::vector<int>& incident(int v) const { return adj_[v]; }

    // Returns an OrientationState with the initial orientation:
    // every edge {u,v} with u < v is set u → v (dir = true).
    // All algorithms must start from this to ensure fair comparison.
    OrientationState fresh_orientation() const;

    OrientationState snapshot(const OrientationState& s) const { return s; }

private:

    int n_ = 0;
    std::vector<Edge> edges_;               // edge list, u < v enforced
    std::vector<std::vector<int>> adj_;     // adj_[v] = incident edge IDs

    void build_adj();
    void add_edge(int u, int v);
};