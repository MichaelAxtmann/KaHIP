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

#include <queue>
#include <stdlib.h>
#include <chrono>

#include "data_structure/flow_graph.h"
#include "data_structure/graph_access.h"
#include "flow_graph_io.h"

#define AGGRESSIVE_RELABELING
// Perform highest level first (instead of fifo).
#define HIGHEST_LEVEL_FIRST
#define TWO_PHASE_APPROACH

struct NodeData {
    NodeData(NodeID node_cnt)
        : access(0), level(0), current_ptr(0)
#ifdef AGGRESSIVE_RELABELING
        , next_level(2* node_cnt - 1)
#endif
    {}
        
    FlowType access;
    NodeID level;
    EdgeID current_ptr;
#ifdef AGGRESSIVE_RELABELING
    NodeID next_level;
#endif
};

#ifdef HIGHEST_LEVEL_FIRST
struct ActiveNode {
    ActiveNode() {}
    ActiveNode(NodeID v, NodeID level)
        : v(v), level(level) {}
    NodeID v;
    NodeID level;

    bool operator()(const ActiveNode& node1, const ActiveNode& node2) {
        return node1.level < node2.level;
    }
};
using ActiveNodeContainer = std::priority_queue<ActiveNode, std::vector<ActiveNode>,
                                                ActiveNode>;
#else
using ActiveNodeContainer = std::queue<NodeID>;
#endif

class PreflowPush {
public:
    FlowType preflow_push(flow_graph& G, NodeID source, NodeID sink);

private:
    // Executes the preflow push algorithm for nodes on level < node_cnt
    // This method adds active nodes to the container small_active_nodes_
    // or to the container active_nodes_ depending on their level.
    void low_level_preflow_push(flow_graph& G, NodeID source, NodeID sink);

    // Executes the preflow push algorithm for nodes on arbitrary levels
    // stored in the active_nodes_ container.
    FlowType remaining_level_preflow_push(flow_graph& G, NodeID source, NodeID sink);

    EdgeID get_next_eligible_edge(const flow_graph& G, NodeID v);

    void relabel(NodeID v);

    // Pushes flow over edge e in the first phase.
    // If the target node gets activated, we add the target node to
    // the container small_active_nodes_ or the container active_nodes_
    // depending on the level of the target node.
    void low_level_push(flow_graph& G, NodeID v, EdgeID e, FlowType flow);
    
    void push(flow_graph& G, NodeID v, EdgeID e, FlowType flow);

    // Returns maximal residual flow based on remaining access.
    FlowType max_allowed_res_flow(const flow_graph& G, NodeID v, EdgeID e) const;

    FlowType get_outgoing_flow(const flow_graph& G, EdgeID e) const;

    NodeID                node_cnt_;
    NodeID                source_;
    NodeID                sink_;
    std::vector<NodeData> node_data_;

    // Use this container for nodes on level < node_cnt
    // if the two level approach is performed.
    ActiveNodeContainer small_active_nodes_;
    // Two phase approach: active_nodes_ stores active nodes on level < node_cnt.
    // One phase approach: active_nodes_ stores all active nodes.
    ActiveNodeContainer active_nodes_;
};

inline
FlowType PreflowPush::preflow_push(flow_graph& G, NodeID source, NodeID sink) {
        
    node_cnt_           = G.number_of_nodes();
    node_data_          = std::vector<NodeData>(node_cnt_, node_cnt_);
    active_nodes_       = ActiveNodeContainer();
    source_             = source;
    sink_               = sink;
#ifdef TWO_PHASE_APPROACH
    small_active_nodes_ = ActiveNodeContainer();
#endif

    // init

    // saturate outgoing edges of source
    forall_out_edges(G, e, source) {
        const FlowType res_flow = G.getEdgeResCapacity(source, e);
        const NodeID target = G.getEdgeTarget(source, e);
        node_data_[target].level = 1;

#ifdef TWO_PHASE_APPROACH
        low_level_push(G, source, e, res_flow);
#else
        push(G, source, e, res_flow);
#endif
    } endfor
          node_data_[source].level = G.number_of_nodes();

#ifdef TWO_PHASE_APPROACH
    low_level_preflow_push(G, source, sink);
#endif
    return remaining_level_preflow_push(G, source, sink);
}    

inline
void PreflowPush::low_level_preflow_push(flow_graph& G, NodeID source, NodeID sink) {

    while (!small_active_nodes_.empty()) {
#ifdef HIGHEST_LEVEL_FIRST
        const NodeID active_node = small_active_nodes_.top().v;
#else
        const NodeID active_node = small_active_nodes_.front();
#endif
        small_active_nodes_.pop();

        // Request first eligible edge.
        EdgeID e_eligible = get_next_eligible_edge(G, active_node);
            
        // Push access as long as possible.
        while (node_data_[active_node].access > 0 &&
               G.isValidEdge(active_node, e_eligible)) {
            const EdgeID flow = max_allowed_res_flow(G, active_node, e_eligible);
            low_level_push(G, active_node, e_eligible, flow);

            // Request next eligible edge.
            e_eligible = get_next_eligible_edge(G, active_node);
        }

        // No eligible edge available but access remains.
        if (node_data_[active_node].access > 0)
        {
            relabel(active_node);

            // Add node to list of active nodes.
#ifdef HIGHEST_LEVEL_FIRST
            if (node_data_[active_node].level < node_cnt_) {
                small_active_nodes_.emplace(active_node, node_data_[active_node].level);
            } else {
                active_nodes_.emplace(active_node, node_data_[active_node].level);
            }
#else
            if (node_data_[active_node].level < node_cnt_) {
                small_active_nodes_.emplace(active_node);
            } else {
                active_nodes_.emplace(active_node);
            }
#endif
        }
    }
}    

inline
FlowType PreflowPush::remaining_level_preflow_push(flow_graph& G, NodeID source, NodeID sink) {

    while (!active_nodes_.empty()) {
#ifdef HIGHEST_LEVEL_FIRST
        const NodeID active_node = active_nodes_.top().v;
#else
        const NodeID active_node = active_nodes_.front();
#endif
        active_nodes_.pop();

        // Request first eligible edge.
        EdgeID e_eligible = get_next_eligible_edge(G, active_node);

        // Push access as long as possible.
        while (node_data_[active_node].access > 0 && G.isValidEdge(active_node, e_eligible)) {
            const EdgeID flow = max_allowed_res_flow(G, active_node, e_eligible);
            push(G, active_node, e_eligible, flow);

            // Request next eligible edge.
            e_eligible = get_next_eligible_edge(G, active_node);
        }

        // No eligible edge available but access remains.
        if (node_data_[active_node].access > 0)
        {
            relabel(active_node);

            // Add node to list of active nodes.
#ifdef HIGHEST_LEVEL_FIRST
            active_nodes_.emplace(active_node, node_data_[active_node].level);
#else
            active_nodes_.emplace(active_node);
#endif
        }
    }

    // Calculate max flow.
    return get_outgoing_flow(G, source_);
}                

inline
EdgeID PreflowPush::get_next_eligible_edge(const flow_graph& G,
                                               NodeID v) {
    const EdgeID e_end = G.get_first_invalid_edge(v);
    EdgeID& e_curr = node_data_[v].current_ptr;
    
    while(e_curr != e_end) {
        const auto target = G.getEdgeTarget(v, e_curr);
        const bool saturated = G.isSaturated(v, e_curr);
        if (!saturated && node_data_[target].level < node_data_[v].level) {
            break;
        } else {
#ifdef AGGRESSIVE_RELABELING
            if (!saturated) {
                assert(node_data_[v].next_level > node_data_[v].level);
                const auto level_update = std::min(node_data_[v].next_level,
                                                   node_data_[target].level + 1);
                node_data_[v].next_level = level_update;
            }
#endif
            e_curr++;
        }
    }
    return e_curr;
};

#ifdef AGGRESSIVE_RELABELING
// Aggressive relabeling.
inline
void PreflowPush::relabel(NodeID v) {
    node_data_[v].level = node_data_[v].next_level;
    node_data_[v].next_level = 2 * node_cnt_ - 1;
    node_data_[v].current_ptr = 0;
}

#else
// Default relabeling.
inline
void PreflowPush::relabel(NodeID v) {
    node_data_[v].level++;
    node_data_[v].current_ptr = 0;
}
#endif

inline
void PreflowPush::low_level_push(flow_graph& G, NodeID v, EdgeID e, FlowType flow) {
    auto target = G.getEdgeTarget(v, e);

    const bool non_pos_access = node_data_[target].access <= 0;

    // push flow
    G.increaseEdgeFlow(v, e, flow);
    node_data_[target].access += flow;
    node_data_[v].access -= flow;
    
    // Activate target if target became a node with positive access.
    // source and sink never become active nodes.
    if (target != source_ && target != sink_ &&
        non_pos_access &&
        node_data_[target].access > 0) {
#ifdef HIGHEST_LEVEL_FIRST
        if (node_data_[target].level < node_cnt_) {
            small_active_nodes_.emplace(target, node_data_[target].level);
        } else {
            active_nodes_.emplace(target, node_data_[target].level);
        }
#else
        if (node_data_[target].level < node_cnt_) {
            small_active_nodes_.push(target);
        } else {
            active_nodes_.push(target);
        }
#endif
    }
}

inline
void PreflowPush::push(flow_graph& G, NodeID v, EdgeID e, FlowType flow) {
    auto target = G.getEdgeTarget(v, e);

    const bool non_pos_access = node_data_[target].access <= 0;

    // push flow
    G.increaseEdgeFlow(v, e, flow);
    node_data_[target].access += flow;
    node_data_[v].access -= flow;
    
    // Activate target if target became a node with positive access.
    // source and sink never become active nodes.
    if (target != source_ && target != sink_ &&
        non_pos_access &&
        node_data_[target].access > 0) {
#ifdef HIGHEST_LEVEL_FIRST
        active_nodes_.emplace(target, node_data_[target].level);
#else
        active_nodes_.push(target);
#endif
    }
}

inline
FlowType PreflowPush::max_allowed_res_flow(const flow_graph& G, NodeID v, EdgeID e) const {
    const auto acc = node_data_[v].access;
    const auto res_flow = G.getEdgeResCapacity(v, e);
    return std::min(acc, res_flow);
}

inline
FlowType PreflowPush::get_outgoing_flow(const flow_graph& G, EdgeID edge) const {
    FlowType flow = 0;
    forall_out_edges(G, e, edge) {
        flow += G.getEdgeFlow(source_, e);
    } endfor;
    return flow;
}

using namespace std::chrono;
int main(int argc, char *argv[]) {
    if (argc > 1) {
    }
    else {
        std::cout << "File path required." << std::endl;
        return 1;
    }

    std::string file_name = argv[1];

    // Read graph.
    flow_graph G;
    NodeID     source;
    NodeID     sink;
    flow_graph_io::readDIMACS(G, file_name, source, sink);
    std::cout << "source: " << source << std::endl;
    std::cout << "sink: " << sink << std::endl;

    // Info output.
    std::cout << "Preflow push." << std::endl;
#ifdef AGGRESSIVE_RELABELING
    std::cout << "\tAGGRESSIVE_RELABELING" << std::endl;
#endif
#ifdef HIGHEST_LEVEL_FIRST
    std::cout << "\tHIGHEST_LEVEL_FIRST" << std::endl;
#endif
#ifdef TWO_PHASE_APPROACH
    std::cout << "\tTWO_PHASE_APPROACH" << std::endl;
#endif

    // Execute algorithm.
    std::cout << "Pending..." << std::endl;
    auto start_time = system_clock::now();
    FlowType flow = PreflowPush().preflow_push(G, source, sink);
    auto end_time = system_clock::now();
    std::cout << "Terminated." << std::endl;
    auto diff = duration_cast<milliseconds>(end_time - start_time).count();

    // Info output.
    std::cout << "time:\t\t" << diff << " ms" << std::endl;
    // max flow has been found
    std::cout << "Max flow:\t" << flow << std::endl;
    return 0;
}
