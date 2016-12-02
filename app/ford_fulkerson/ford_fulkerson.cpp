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

#include <vector>
#include <iostream>

#include <stdlib.h>

#include "data_structure/flow_graph.h"
#include "flow_graph_io.h"

#include "res_graph_dfs.hpp"

int main(int argc, char *argv[]) {
    if (argc > 1) {
    }
    else {
        std::cout << "File path required." << std::endl;
        return 1;
    }

    std::string file_name = argv[1];

    flow_graph G;
    NodeID     source;
    NodeID     sink;
    flow_graph_io::readDIMACS(G, file_name, source, sink);
    std::cout << "source: " << source << std::endl;
    std::cout << "sink: " << sink << std::endl;
 
    FlowType            flow = 0;
    std::vector<NodeID> nodes;
    std::vector<EdgeID> edges;

    // find augmenting path
    auto aug_flow = res_graph_dfs(G, nodes, edges, source, sink);
    while (aug_flow > 0) {
        // ... apply path and try to find a new path
        flow += aug_flow;
        // apply flow to graph
        for (size_t edge_index = 0; edge_index != edges.size(); ++edge_index) {
            auto e = edges[edge_index];
            auto source = nodes[edge_index];
            G.increaseEdgeFlow(source, e, aug_flow);
        }
        nodes.clear();
        edges.clear();
        aug_flow = res_graph_dfs(G, nodes, edges, source, sink);
    }
    
    // max flow has been found
    std::cout << "Max flow: " << flow << std::endl;
    return 0;
}
