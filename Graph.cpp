#include "Graph.h"

#include <fstream>
#include <sstream>
#include <unordered_set>
#include <algorithm>
#include <stdexcept>
#include <iostream>


using namespace std;


// Canonical edge key: always store with smaller endpoint first
static pair<int, int> canonical(int u, int v) {
    return u < v ? make_pair(u, v) : make_pair(v, u);
}


void Graph::add_edge(int u, int v) {
    // Enforce u < v convention
    if (u > v) {
        swap(u, v);
    }
    int eid = (int)edges_.size();
    edges_.push_back({u, v});
    adj_[u].push_back(eid);
    adj_[v].push_back(eid);
}


void Graph::build_adj() {
    adj_.assign(n_, {});
}


Graph Graph::from_edge_list(const std::string& path) {
    ifstream file(path);
    if (!file.is_open()) {
        throw runtime_error("Cannot open graph file: " + path);
    }

    Graph g;

    vector<pair<int, int>> raw_edges;
    int max_id = -1;

    string line;
    while (getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        istringstream iss(line);
        
        int u, v;
        if (!(iss >> u >> v)) continue;
        if (u == v) continue;

        int lo = min(u, v);
        int hi = max(u, v);

        raw_edges.push_back({lo, hi});
        max_id = max(max_id, hi);
    }

    if (max_id < 0) throw runtime_error("Graph file is empty or has no valid edges.");

    g.n_ = max_id + 1;
    g.build_adj();

    sort(raw_edges.begin(), raw_edges.end());
    raw_edges.erase(unique(raw_edges.begin(), raw_edges.end()), raw_edges.end());

    g.edges_.reserve(raw_edges.size());
    for (auto& [u, v] : raw_edges) {
        g.add_edge(u, v);
    }

    cout << "Loaded graph: n = " << g.n() << " m = " << g.m() << " density = " << g.density() << "\n";

    return g;
}


Graph Graph::from_pairs(int num_vertices, const vector<pair<int, int>>& pairs) {
    Graph g;
    g.n_ = num_vertices;
    g.build_adj();

    vector<pair<int, int>> canon;
    canon.reserve(pairs.size());
    for (auto [u, v] : pairs) {
        if (u == v) continue;
        canon.push_back(canonical(u, v));
    }

    sort(canon.begin(), canon.end());
    canon.erase(unique(canon.begin(), canon.end()), canon.end());

    for (auto [u, v] : canon) {
        g.add_edge(u, v);
    }   

    return g;
}


OrientationState Graph::fresh_orientation() const {
    OrientationState s;

    s.dir.assign(m(), true);
    s.outdeg.assign(n_, 0);

    for (const Edge& e : edges_) {
        s.outdeg[e.u]++;
    }

    return s;
}

