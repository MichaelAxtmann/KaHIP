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

    // TODO ... implement dfs
    return res_flow;
}
