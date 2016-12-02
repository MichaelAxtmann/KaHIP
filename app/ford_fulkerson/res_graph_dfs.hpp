/******************************************************************************
 * flow_graph.h 
 *
 ******************************************************************************
 * Copyright (C) 2016 Michael Axtmann <michael.axtmann@kit.edu>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <limits>
#include <vector>
#include <vector>
#include <assert.h>

#include "data_structure/flow_graph.h"

template<class graph>
FlowType res_graph_dfs(graph& G,
                       std::vector<NodeID>& nodes, std::vector<EdgeID>& edges,
                       const NodeID s, const NodeID t) {
    assert(s < G.number_of_nodes());
    assert(t < G.number_of_nodes());
    std::vector<bool> marked(G.number_of_nodes(), false);
    nodes.clear();
    edges.clear();

    // Guarantee that there is a valid outgoing edge with remaining flow.
    auto e = G.get_first_flow_edge(s);
    auto v = s;
    if (e == G.get_first_invalid_edge(s) || s == t) {
        return 0;
    }
    nodes.push_back(v);
    edges.push_back(e);

    while(!nodes.empty()) {
        v = nodes.back();
        e = edges.back();
        assert(v != t);
        assert(e != G.get_first_invalid_edge(v));
        
        auto w = G.getEdgeTarget(v, e);
        auto e_w = G.get_first_flow_edge(w);
        auto e_w_end = G.get_first_invalid_edge(w);

        if (!marked[w] && e_w != e_w_end) {
            nodes.push_back(w);
            edges.push_back(e_w);
            marked[w] = true;
        } else {
            e = G.get_next_flow_edge(v, e);
            if (e != G.get_first_invalid_edge(v)) {
                edges.back() = e;
            } else {
                nodes.pop_back();
                edges.pop_back();
            }
        }
        // Breakthrough. Stop dfs.
        if (w == t) {
            break;
        }
    }

    // Calculate flow.
    assert(edges.size() == nodes.size());
    if (nodes.empty()) {
        return 0;
    }
    FlowType res_flow = std::numeric_limits<FlowType>::max();
    for (size_t node_idx = 0; node_idx != nodes.size(); ++node_idx) {
        auto v = nodes[node_idx];
        auto e = edges[node_idx];
        auto res_edge_flow = G.getEdgeResCapacity(v,e);
        res_flow = std::min(res_flow, res_edge_flow);
    }
    return res_flow;
}
